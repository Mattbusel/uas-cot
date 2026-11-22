//! `net::TlsSender` against a local rustls server that requires a client
//! certificate, the way a TAK Server TLS streaming input (8089) does.
#![cfg(feature = "tls")]

use rcgen::{BasicConstraints, Certificate, CertificateParams, DnType, IsCa, KeyPair};
use std::io::Read;
use std::net::{SocketAddr, TcpListener};
use std::sync::Arc;
use uas_cot::net::{TlsConfig, TlsSender};
use uas_cot::rustls::{self, pki_types::PrivateKeyDer, server::WebPkiClientVerifier};
use uas_cot::Event;

struct Ca {
    cert: Certificate,
    key: KeyPair,
}

fn ca(name: &str) -> Ca {
    let key = KeyPair::generate().unwrap();
    let mut p = CertificateParams::new(Vec::<String>::new()).unwrap();
    p.distinguished_name.push(DnType::CommonName, name);
    p.is_ca = IsCa::Ca(BasicConstraints::Unconstrained);
    let cert = p.self_signed(&key).unwrap();
    Ca { cert, key }
}

/// A leaf signed by `ca`: (cert PEM, key PEM, cert DER, key DER).
fn leaf(ca: &Ca, names: &[&str], cn: &str) -> (String, String, Vec<u8>, Vec<u8>) {
    let key = KeyPair::generate().unwrap();
    let mut p =
        CertificateParams::new(names.iter().map(|s| s.to_string()).collect::<Vec<_>>()).unwrap();
    p.distinguished_name.push(DnType::CommonName, cn);
    let cert = p.signed_by(&key, &ca.cert, &ca.key).unwrap();
    (
        cert.pem(),
        key.serialize_pem(),
        cert.der().to_vec(),
        key.serialize_der(),
    )
}

/// A one-connection TLS server with client certificate auth against
/// `client_ca`. Returns its address and a handle yielding what it read
/// (or the handshake error text).
fn server(
    client_ca: &Ca,
    server_names: &[&str],
    server_cn: &str,
    want: usize,
) -> (SocketAddr, std::thread::JoinHandle<Result<String, String>>) {
    let provider = Arc::new(rustls::crypto::ring::default_provider());
    let (_, _, cert_der, key_der) = leaf(client_ca, server_names, server_cn);
    let mut roots = rustls::RootCertStore::empty();
    roots.add(client_ca.cert.der().clone()).unwrap();
    let verifier = WebPkiClientVerifier::builder_with_provider(Arc::new(roots), provider.clone())
        .build()
        .unwrap();
    let cfg = rustls::ServerConfig::builder_with_provider(provider)
        .with_safe_default_protocol_versions()
        .unwrap()
        .with_client_cert_verifier(verifier)
        .with_single_cert(
            vec![cert_der.into()],
            PrivateKeyDer::try_from(key_der).unwrap(),
        )
        .unwrap();
    let l = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = l.local_addr().unwrap();
    let h = std::thread::spawn(move || {
        let (sock, _) = l.accept().map_err(|e| e.to_string())?;
        let conn = rustls::ServerConnection::new(Arc::new(cfg)).map_err(|e| e.to_string())?;
        let mut s = rustls::StreamOwned::new(conn, sock);
        let mut buf = vec![0u8; want];
        s.read_exact(&mut buf).map_err(|e| e.to_string())?;
        String::from_utf8(buf).map_err(|e| e.to_string())
    });
    (addr, h)
}

fn event() -> Event {
    let mut e = Event::new("tls-test", "a-f-A-M-F-Q", 1_700_000_000_000_000, 30);
    e.callsign = Some("TLS".into());
    e
}

#[test]
fn sends_with_client_certificate() {
    let ca = ca("TAK test CA");
    let (cpem, kpem, _, _) = leaf(&ca, &[], "uas-cot client");
    let xml = event().to_xml();
    let (addr, h) = server(&ca, &["localhost"], "takserver", xml.len() * 2);
    let tls =
        TlsConfig::from_pem(ca.cert.pem().as_bytes(), cpem.as_bytes(), kpem.as_bytes()).unwrap();
    let mut tx = TlsSender::connect(&format!("localhost:{}", addr.port()), &tls).unwrap();
    tx.send(&event()).unwrap();
    tx.send(&event()).unwrap();
    assert_eq!(h.join().unwrap().unwrap(), xml.repeat(2));
    tx.close().unwrap();
}

#[test]
fn name_mismatch_fails_unless_any_name() {
    let ca = ca("TAK test CA");
    let (cpem, kpem, _, _) = leaf(&ca, &[], "uas-cot client");
    let xml = event().to_xml();

    // Certificate names takserver.example; we dial 127.0.0.1.
    let (addr, h) = server(&ca, &["takserver.example"], "takserver.example", xml.len());
    let strict =
        TlsConfig::from_pem(ca.cert.pem().as_bytes(), cpem.as_bytes(), kpem.as_bytes()).unwrap();
    let err = TlsSender::connect(&addr.to_string(), &strict).unwrap_err();
    assert!(err.to_string().contains("not valid for name"), "{err}");
    let _ = h.join();

    let (addr, h) = server(&ca, &["takserver.example"], "takserver.example", xml.len());
    let any =
        TlsConfig::from_pem_any_name(ca.cert.pem().as_bytes(), cpem.as_bytes(), kpem.as_bytes())
            .unwrap();
    let mut tx = TlsSender::connect(&addr.to_string(), &any).unwrap();
    tx.send(&event()).unwrap();
    assert_eq!(h.join().unwrap().unwrap(), xml);

    // connect_named checks against the name the certificate carries.
    let (addr, h) = server(&ca, &["takserver.example"], "takserver.example", xml.len());
    let mut tx = TlsSender::connect_named(addr, "takserver.example", &strict).unwrap();
    tx.send(&event()).unwrap();
    assert_eq!(h.join().unwrap().unwrap(), xml);
}

#[test]
fn any_name_still_rejects_an_untrusted_ca() {
    let real = ca("TAK test CA");
    let other = ca("someone else");
    let (cpem, kpem, _, _) = leaf(&real, &[], "uas-cot client");
    // The server's certificate (and client CA) come from `other`.
    let (addr, h) = server(&other, &["localhost"], "takserver", 1);
    let any =
        TlsConfig::from_pem_any_name(real.cert.pem().as_bytes(), cpem.as_bytes(), kpem.as_bytes())
            .unwrap();
    let err = TlsSender::connect(&format!("localhost:{}", addr.port()), &any).unwrap_err();
    assert!(err.to_string().to_lowercase().contains("unknown"), "{err}");
    let _ = h.join();
}

#[test]
fn bad_pem_is_an_error() {
    let ca = ca("TAK test CA");
    let (cpem, kpem, _, _) = leaf(&ca, &[], "c");
    let e = TlsConfig::from_pem(b"not pem", cpem.as_bytes(), kpem.as_bytes()).unwrap_err();
    assert!(e.to_string().contains("CA PEM"), "{e}");
    let e = TlsConfig::from_pem(ca.cert.pem().as_bytes(), b"", kpem.as_bytes()).unwrap_err();
    assert!(e.to_string().contains("client certificate"), "{e}");
    let e = TlsConfig::from_pem(ca.cert.pem().as_bytes(), cpem.as_bytes(), b"").unwrap_err();
    assert!(e.to_string().contains("client key"), "{e}");
}
