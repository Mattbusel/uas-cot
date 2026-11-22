//! Sending events: UDP (the TAK SA multicast group, or a TAK Server UDP
//! input), TCP (a TAK Server plain streaming input, usually port 8087) and,
//! with the `tls` feature, TLS with a client certificate (`TlsSender`, the
//! TAK Server streaming input on port 8089). Without the feature, wrap your
//! own TLS stream and call [`write_event`].

use crate::Event;
use std::io::{self, Write};
use std::net::{SocketAddr, TcpStream, ToSocketAddrs, UdpSocket};

#[cfg(feature = "tls")]
mod tls;
#[cfg(feature = "tls")]
pub use tls::{TlsConfig, TlsSender};

/// The TAK situational awareness multicast group, `239.2.3.1:6969`.
pub const SA_MULTICAST: &str = "239.2.3.1:6969";

/// Sends events as UDP datagrams, one event per datagram.
#[derive(Debug)]
pub struct UdpSender {
    socket: UdpSocket,
    dest: SocketAddr,
}

impl UdpSender {
    /// A sender to `dest` (unicast or multicast).
    pub fn new(dest: impl ToSocketAddrs) -> io::Result<Self> {
        let dest = dest
            .to_socket_addrs()?
            .next()
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "no address"))?;
        let bind: SocketAddr = if dest.is_ipv4() {
            "0.0.0.0:0".parse().expect("valid")
        } else {
            "[::]:0".parse().expect("valid")
        };
        let socket = UdpSocket::bind(bind)?;
        if let SocketAddr::V4(v4) = dest {
            if v4.ip().is_multicast() {
                socket.set_multicast_ttl_v4(1)?;
            }
        }
        Ok(UdpSender { socket, dest })
    }

    /// A sender to the SA multicast group ([`SA_MULTICAST`]), TTL 1.
    pub fn sa_multicast() -> io::Result<Self> {
        UdpSender::new(SA_MULTICAST)
    }

    /// Sends one event.
    pub fn send(&self, event: &Event) -> io::Result<()> {
        self.socket
            .send_to(event.to_xml().as_bytes(), self.dest)
            .map(|_| ())
    }
}

/// Writes one event to a stream (TCP or TLS): the XML document, as TAK
/// streaming inputs expect.
pub fn write_event(w: &mut impl Write, event: &Event) -> io::Result<()> {
    w.write_all(event.to_xml().as_bytes())
}

/// Sends events over one TCP connection.
#[derive(Debug)]
pub struct TcpSender {
    stream: TcpStream,
}

impl TcpSender {
    /// Connects to a TAK Server streaming input, for example `takserver:8087`.
    pub fn connect(addr: impl ToSocketAddrs) -> io::Result<Self> {
        let stream = TcpStream::connect(addr)?;
        stream.set_nodelay(true)?;
        Ok(TcpSender { stream })
    }

    /// Sends one event.
    pub fn send(&mut self, event: &Event) -> io::Result<()> {
        write_event(&mut self.stream, event)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Read;

    #[test]
    fn udp_and_tcp_deliver_the_xml() {
        let e = Event::new("net-test", "a-f-A", 0, 1);
        let rx = UdpSocket::bind("127.0.0.1:0").unwrap();
        UdpSender::new(rx.local_addr().unwrap())
            .unwrap()
            .send(&e)
            .unwrap();
        let mut buf = [0u8; 2048];
        let n = rx.recv(&mut buf).unwrap();
        assert_eq!(&buf[..n], e.to_xml().as_bytes());

        let l = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = l.local_addr().unwrap();
        let t = std::thread::spawn(move || {
            let (mut s, _) = l.accept().unwrap();
            let mut v = String::new();
            s.read_to_string(&mut v).unwrap();
            v
        });
        {
            let mut tx = TcpSender::connect(addr).unwrap();
            tx.send(&e).unwrap();
            tx.send(&e).unwrap();
        }
        assert_eq!(t.join().unwrap(), e.to_xml().repeat(2));
    }
}
