//! TLS streaming to a TAK Server (feature `tls`, rustls with the ring
//! provider, no OpenSSL).

use crate::Event;
use rustls::client::danger::{HandshakeSignatureValid, ServerCertVerified, ServerCertVerifier};
use rustls::client::WebPkiServerVerifier;
use rustls::crypto::CryptoProvider;
use rustls::pki_types::pem::PemObject;
use rustls::pki_types::{CertificateDer, PrivateKeyDer, ServerName, UnixTime};
use rustls::{
    CertificateError, ClientConfig, ClientConnection, DigitallySignedStruct, RootCertStore,
};
use rustls::{SignatureScheme, StreamOwned};
use std::io::{self, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::path::Path;
use std::sync::Arc;

fn invalid(msg: impl std::fmt::Display) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, msg.to_string())
}

fn provider() -> Arc<CryptoProvider> {
    Arc::new(rustls::crypto::ring::default_provider())
}

/// How to reach a TAK Server TLS streaming input (usually port 8089): the
/// CA that signed the server certificate, and your client certificate and
/// key. TAK Server requires the client certificate by default.
///
/// TAK hands out `.p12` files. Convert them to PEM once with OpenSSL (the
/// default TAK password is often `atakatak`; OpenSSL 3 needs `-legacy` for
/// the older encryption TAK uses):
///
/// ```text
/// openssl pkcs12 -legacy -in truststore-root.p12 -nokeys -out ca.pem
/// openssl pkcs12 -legacy -in user.p12 -clcerts -nokeys -out user.pem
/// openssl pkcs12 -legacy -in user.p12 -nocerts -nodes -out user.key
/// ```
///
/// For anything else (PKCS#12 directly, platform trust stores, custom
/// verification) build a [`rustls::ClientConfig`] yourself and use
/// [`TlsConfig::from_rustls`].
#[derive(Clone, Debug)]
pub struct TlsConfig {
    config: Arc<ClientConfig>,
}

/// Accepts a certificate whose chain verifies against the configured CA but
/// whose names do not match the address dialled. Every other failure
/// (unknown CA, expired, bad signature, wrong key usage) still fails.
#[derive(Debug)]
struct AnyName(Arc<WebPkiServerVerifier>);

impl ServerCertVerifier for AnyName {
    fn verify_server_cert(
        &self,
        end_entity: &CertificateDer<'_>,
        intermediates: &[CertificateDer<'_>],
        server_name: &ServerName<'_>,
        ocsp_response: &[u8],
        now: UnixTime,
    ) -> Result<ServerCertVerified, rustls::Error> {
        // WebPkiServerVerifier checks the chain before the name, so a name
        // error means the chain is good.
        match self
            .0
            .verify_server_cert(end_entity, intermediates, server_name, ocsp_response, now)
        {
            Err(rustls::Error::InvalidCertificate(
                CertificateError::NotValidForName | CertificateError::NotValidForNameContext { .. },
            )) => Ok(ServerCertVerified::assertion()),
            r => r,
        }
    }

    fn verify_tls12_signature(
        &self,
        message: &[u8],
        cert: &CertificateDer<'_>,
        dss: &DigitallySignedStruct,
    ) -> Result<HandshakeSignatureValid, rustls::Error> {
        self.0.verify_tls12_signature(message, cert, dss)
    }

    fn verify_tls13_signature(
        &self,
        message: &[u8],
        cert: &CertificateDer<'_>,
        dss: &DigitallySignedStruct,
    ) -> Result<HandshakeSignatureValid, rustls::Error> {
        self.0.verify_tls13_signature(message, cert, dss)
    }

    fn supported_verify_schemes(&self) -> Vec<SignatureScheme> {
        self.0.supported_verify_schemes()
    }
}

impl TlsConfig {
    /// From PEM bytes: the CA certificate(s) to trust, your client
    /// certificate chain, and its private key (PKCS#8, PKCS#1 or SEC1).
    /// The server certificate must name the host you connect to.
    pub fn from_pem(ca: &[u8], client_cert: &[u8], client_key: &[u8]) -> io::Result<Self> {
        Self::build(ca, client_cert, client_key, true)
    }

    /// [`from_pem`](Self::from_pem), reading the three PEM files.
    pub fn from_pem_files(
        ca: impl AsRef<Path>,
        client_cert: impl AsRef<Path>,
        client_key: impl AsRef<Path>,
    ) -> io::Result<Self> {
        Self::from_pem(
            &std::fs::read(ca)?,
            &std::fs::read(client_cert)?,
            &std::fs::read(client_key)?,
        )
    }

    /// Like [`from_pem`](Self::from_pem), but a server certificate that does
    /// not name the host you dialled is accepted, as long as it chains to
    /// `ca`. Use it when you reach the server by an IP address or an
    /// internal name its certificate does not list: rustls (like browsers)
    /// does not fall back to the certificate's common name, and a TAK
    /// Server certificate made with only a CN fails the name check. An
    /// untrusted, expired or self-signed-by-someone-else certificate still
    /// fails.
    pub fn from_pem_any_name(ca: &[u8], client_cert: &[u8], client_key: &[u8]) -> io::Result<Self> {
        Self::build(ca, client_cert, client_key, false)
    }

    /// Your own rustls client configuration.
    pub fn from_rustls(config: Arc<ClientConfig>) -> Self {
        TlsConfig { config }
    }

    /// The rustls client configuration in use.
    pub fn rustls_config(&self) -> &Arc<ClientConfig> {
        &self.config
    }

    fn build(ca: &[u8], cert: &[u8], key: &[u8], check_name: bool) -> io::Result<Self> {
        let mut roots = RootCertStore::empty();
        for c in CertificateDer::pem_slice_iter(ca) {
            roots
                .add(c.map_err(|e| invalid(format!("CA PEM: {e}")))?)
                .map_err(|e| invalid(format!("CA certificate: {e}")))?;
        }
        if roots.is_empty() {
            return Err(invalid("CA PEM: no certificate found"));
        }
        let chain = CertificateDer::pem_slice_iter(cert)
            .collect::<Result<Vec<_>, _>>()
            .map_err(|e| invalid(format!("client certificate PEM: {e}")))?;
        if chain.is_empty() {
            return Err(invalid("client certificate PEM: no certificate found"));
        }
        let key = PrivateKeyDer::from_pem_slice(key)
            .map_err(|e| invalid(format!("client key PEM: {e}")))?;
        let provider = provider();
        let builder = ClientConfig::builder_with_provider(provider.clone())
            .with_safe_default_protocol_versions()
            .map_err(invalid)?;
        let roots = Arc::new(roots);
        let builder = if check_name {
            builder.with_root_certificates(roots)
        } else {
            let webpki = WebPkiServerVerifier::builder_with_provider(roots, provider)
                .build()
                .map_err(invalid)?;
            builder
                .dangerous()
                .with_custom_certificate_verifier(Arc::new(AnyName(webpki)))
        };
        let config = builder
            .with_client_auth_cert(chain, key)
            .map_err(|e| invalid(format!("client certificate and key: {e}")))?;
        Ok(TlsConfig {
            config: Arc::new(config),
        })
    }
}

/// Sends events over one TLS connection to a TAK Server streaming input,
/// for example `takserver:8089`. The handshake (including the client
/// certificate) completes in `connect`, so a wrong CA or a rejected
/// certificate fails there, not on the first event.
///
/// ```no_run
/// use uas_cot::net::{TlsConfig, TlsSender};
///
/// let tls = TlsConfig::from_pem_files("ca.pem", "user.pem", "user.key")?;
/// let mut tak = TlsSender::connect("takserver.example:8089", &tls)?;
/// tak.send(&uas_cot::Event::new("test-1", "a-f-G", uas_cot::now_us(), 60))?;
/// # Ok::<(), std::io::Error>(())
/// ```
#[derive(Debug)]
pub struct TlsSender {
    stream: StreamOwned<ClientConnection, TcpStream>,
}

/// The host part of `host:port`, `[v6]:port` or a bare host.
fn host_of(addr: &str) -> &str {
    if let Some(rest) = addr.strip_prefix('[') {
        return rest.split(']').next().unwrap_or(rest);
    }
    match addr.rsplit_once(':') {
        Some((h, p)) if !h.contains(':') && p.chars().all(|c| c.is_ascii_digit()) => h,
        _ => addr,
    }
}

impl TlsSender {
    /// Connects to `addr` (`host:port`) and checks the server certificate
    /// against `host`.
    pub fn connect(addr: &str, config: &TlsConfig) -> io::Result<Self> {
        Self::connect_named(addr, host_of(addr), config)
    }

    /// Connects to `addr` and checks the server certificate against
    /// `server_name` (a DNS name or an IP address), for a server you reach
    /// by one address and whose certificate names another.
    pub fn connect_named(
        addr: impl ToSocketAddrs,
        server_name: &str,
        config: &TlsConfig,
    ) -> io::Result<Self> {
        let name = ServerName::try_from(server_name.to_string())
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;
        let mut sock = TcpStream::connect(addr)?;
        sock.set_nodelay(true)?;
        let mut conn = ClientConnection::new(config.config.clone(), name)
            .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
        while conn.is_handshaking() {
            conn.complete_io(&mut sock)?;
        }
        Ok(TlsSender {
            stream: StreamOwned::new(conn, sock),
        })
    }

    /// Sends one event and flushes it to the socket.
    pub fn send(&mut self, event: &Event) -> io::Result<()> {
        super::write_event(&mut self.stream, event)?;
        self.stream.flush()
    }

    /// The TLS stream, for reading what the server sends back (TAK Server
    /// streams other clients' events to a connected client).
    pub fn stream_mut(&mut self) -> &mut StreamOwned<ClientConnection, TcpStream> {
        &mut self.stream
    }

    /// Sends a TLS close_notify and flushes it, so the server sees a clean
    /// end of stream rather than a truncated one.
    pub fn close(mut self) -> io::Result<()> {
        self.stream.conn.send_close_notify();
        self.stream.flush()
    }
}

#[cfg(test)]
mod tests {
    use super::host_of;

    #[test]
    fn host_part() {
        assert_eq!(host_of("takserver:8089"), "takserver");
        assert_eq!(host_of("10.0.0.5:8089"), "10.0.0.5");
        assert_eq!(host_of("[fd00::5]:8089"), "fd00::5");
        assert_eq!(host_of("fd00::5"), "fd00::5");
        assert_eq!(host_of("takserver"), "takserver");
    }
}
