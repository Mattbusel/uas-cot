//! uas2cot: drone telemetry files or streams to Cursor on Target.
//!
//! ```text
//! uas2cot klv  <file.klv>   MISB ST 0601 KLV stream (ffmpeg -i in.ts -map 0:d -c copy -f data file.klv)
//! uas2cot tlog <file.tlog>  MAVLink telemetry log (QGroundControl, Mission Planner, MAVProxy)
//! uas2cot pcap <file.pcap>  Wi-Fi monitor-mode capture with Remote ID beacons / NAN (radiotap or 802.11)
//! uas2cot mavlink-udp <bind addr>   live MAVLink over UDP, for example 0.0.0.0:14550
//!
//! Built with the `mavlink-m` feature, `tlog` and `mavlink-udp` also turn
//! MAVLink-M (military.xml) PARTICIPANT_POSITION friendly position reports
//! into CoT. Target, track, fires and engagement messages are dropped.
//!
//! options:
//!   --udp <addr>       send each event as a UDP datagram (239.2.3.1:6969 is the TAK SA multicast group)
//!   --tcp <addr>       send to a TAK Server TCP streaming input, for example takserver:8087
//!   --uid <uid>        platform UID for KLV input
//!   --video <url>      video URL to attach to the KLV platform
//!   --affiliation <c>  u, n, f, a, s or h (defaults: f for KLV and MAVLink, u for Remote ID)
//!   --realtime         replay files at their recorded pace
//! ```
//!
//! Without `--udp` or `--tcp` the events are printed, one XML document per line.

use std::io::{self, Write};
use std::time::Duration;
use uas_cot::net::{TcpSender, UdpSender};
use uas_cot::{fmv, mavlink, rid, Event};

enum Sink {
    Stdout(io::Stdout),
    Udp(UdpSender),
    Tcp(TcpSender),
}

impl Sink {
    fn send(&mut self, e: &Event) -> io::Result<()> {
        match self {
            Sink::Stdout(o) => writeln!(o.lock(), "{}", e.to_xml()),
            Sink::Udp(u) => u.send(e),
            Sink::Tcp(t) => t.send(e),
        }
    }
}

struct Args {
    mode: String,
    input: String,
    sink: Sink,
    uid: Option<String>,
    video: Option<String>,
    affiliation: Option<char>,
    realtime: bool,
}

fn usage() -> ! {
    eprintln!("usage: uas2cot <klv|tlog|pcap|mavlink-udp> <input> [--udp addr | --tcp addr] [--uid uid] [--video url] [--affiliation c] [--realtime]");
    std::process::exit(2)
}

fn parse_args() -> io::Result<Args> {
    let mut it = std::env::args().skip(1);
    let mode = it.next().unwrap_or_else(|| usage());
    let input = it.next().unwrap_or_else(|| usage());
    let mut a = Args {
        mode,
        input,
        sink: Sink::Stdout(io::stdout()),
        uid: None,
        video: None,
        affiliation: None,
        realtime: false,
    };
    while let Some(flag) = it.next() {
        let mut val = || it.next().unwrap_or_else(|| usage());
        match flag.as_str() {
            "--udp" => a.sink = Sink::Udp(UdpSender::new(val())?),
            "--tcp" => a.sink = Sink::Tcp(TcpSender::connect(val())?),
            "--uid" => a.uid = Some(val()),
            "--video" => a.video = Some(val()),
            "--affiliation" => a.affiliation = val().chars().next(),
            "--realtime" => a.realtime = true,
            _ => usage(),
        }
    }
    Ok(a)
}

/// Sleeps so that events go out at the pace of their timestamps.
struct Pacer {
    first: Option<(u64, std::time::Instant)>,
    on: bool,
}

impl Pacer {
    fn wait(&mut self, ts_us: u64) {
        if !self.on {
            return;
        }
        let (t0, w0) = *self.first.get_or_insert((ts_us, std::time::Instant::now()));
        let due = w0 + Duration::from_micros(ts_us.saturating_sub(t0));
        if let Some(d) = due.checked_duration_since(std::time::Instant::now()) {
            std::thread::sleep(d);
        }
    }
}

fn run_klv(a: &mut Args) -> io::Result<usize> {
    let bytes = std::fs::read(&a.input)?;
    let mut opts = fmv::Options {
        uid: a.uid.clone(),
        video_url: a.video.clone(),
        ..Default::default()
    };
    if let Some(c) = a.affiliation {
        opts.affiliation = c;
    }
    let mut pacer = Pacer {
        first: None,
        on: a.realtime,
    };
    let mut n = 0;
    for unit in misb::klv::split(&bytes) {
        let Ok(packet) = misb::st0601::Packet::from_unit(&unit) else {
            continue;
        };
        if let Some(t) = packet.timestamp() {
            pacer.wait(t);
        }
        for e in fmv::events(&packet, &opts) {
            a.sink.send(&e)?;
            n += 1;
        }
    }
    Ok(n)
}

/// Per-stream MAVLink state: vehicle types and, with `mavlink-m`, the
/// MAVLink-M gateway.
struct MavState {
    opts: mavlink::Options,
    types: [Option<u8>; 256],
    #[cfg(feature = "mavlink-m")]
    gw: uas_cot::mavlink_m::Gateway,
}

impl MavState {
    fn new(affiliation: Option<char>) -> Self {
        let mut opts = mavlink::Options::default();
        if let Some(c) = affiliation {
            opts.affiliation = c;
        }
        MavState {
            opts,
            types: [None; 256],
            #[cfg(feature = "mavlink-m")]
            gw: uas_cot::mavlink_m::Gateway::default(),
        }
    }

    /// The event for one frame, if it makes one.
    fn frame(&mut self, frame: &mavlink_tlog::FrameRef<'_>, ts: u64) -> Option<Event> {
        let sys = frame.sys_id();
        match frame.msg_id() {
            mavlink::HEARTBEAT_ID if frame.comp_id() == 1 => {
                self.types[sys as usize] = mavlink::heartbeat_mav_type(frame.payload());
                None
            }
            mavlink::GLOBAL_POSITION_INT_ID => {
                let pos = mavlink::GlobalPositionInt::parse(frame.payload())?;
                mavlink::vehicle(sys, &pos, self.types[sys as usize], ts, &self.opts)
            }
            #[cfg(feature = "mavlink-m")]
            id => self.gw.ingest(sys, id, frame.payload(), ts),
            #[cfg(not(feature = "mavlink-m"))]
            _ => None,
        }
    }
}

/// CRC_EXTRA lookup: the built-in common/ardupilot table, then MAVLink-M.
fn crc_extra(id: u32) -> Option<u8> {
    if let Some(i) = mavlink_tlog::message_info(id) {
        return Some(i.crc_extra);
    }
    #[cfg(feature = "mavlink-m")]
    {
        uas_cot::mavlink_m::crc_extra(id)
    }
    #[cfg(not(feature = "mavlink-m"))]
    {
        None
    }
}

fn run_tlog(a: &mut Args) -> io::Result<usize> {
    let file = std::fs::File::open(&a.input)?;
    let mut reader = mavlink_tlog::TlogReader::new(io::BufReader::new(file));
    let mut st = MavState::new(a.affiliation);
    let mut pacer = Pacer {
        first: None,
        on: a.realtime,
    };
    let mut n = 0;
    while let Some(rec) = reader.next_record()? {
        let (ts, frame) = (rec.timestamp_us, rec.frame);
        if frame
            .check(mavlink_tlog::CrcPolicy::Custom(crc_extra))
            .is_err()
        {
            continue;
        }
        if let Some(e) = st.frame(&frame, ts) {
            pacer.wait(ts);
            a.sink.send(&e)?;
            n += 1;
        }
    }
    Ok(n)
}

fn run_mavlink_udp(a: &mut Args) -> io::Result<usize> {
    let sock = std::net::UdpSocket::bind(&a.input)?;
    let mut st = MavState::new(a.affiliation);
    let mut buf = [0u8; 2048];
    loop {
        let len = sock.recv(&mut buf)?;
        let mut data = &buf[..len];
        while !data.is_empty() {
            let Ok(frame) = mavlink_tlog::FrameRef::parse_unchecked(data) else {
                break;
            };
            data = &data[frame.len()..];
            if frame
                .check(mavlink_tlog::CrcPolicy::Custom(crc_extra))
                .is_err()
            {
                continue;
            }
            if let Some(e) = st.frame(&frame, uas_cot::now_us()) {
                a.sink.send(&e)?;
            }
        }
    }
}

/// (linktype, timestamp us, packet) from a classic little-endian pcap.
fn read_pcap(b: &[u8]) -> io::Result<Vec<(u32, u64, &[u8])>> {
    let bad = || io::Error::new(io::ErrorKind::InvalidData, "not a little-endian pcap file");
    if b.len() < 24 {
        return Err(bad());
    }
    let nanos = match b[..4] {
        [0xd4, 0xc3, 0xb2, 0xa1] => false,
        [0x4d, 0x3c, 0xb2, 0xa1] => true,
        _ => return Err(bad()),
    };
    let lt = u32::from_le_bytes([b[20], b[21], b[22], b[23]]);
    let mut off = 24;
    let mut out = Vec::new();
    while off + 16 <= b.len() {
        let sec = u32::from_le_bytes(b[off..off + 4].try_into().expect("4 bytes")) as u64;
        let frac = u32::from_le_bytes(b[off + 4..off + 8].try_into().expect("4 bytes")) as u64;
        let cl = u32::from_le_bytes(b[off + 8..off + 12].try_into().expect("4 bytes")) as usize;
        off += 16;
        let Some(pkt) = b.get(off..off + cl) else {
            break;
        };
        out.push((
            lt,
            sec * 1_000_000 + if nanos { frac / 1000 } else { frac },
            pkt,
        ));
        off += cl;
    }
    Ok(out)
}

fn run_pcap(a: &mut Args) -> io::Result<usize> {
    let bytes = std::fs::read(&a.input)?;
    let mut opts = rid::Options::default();
    if let Some(c) = a.affiliation {
        opts.affiliation = c;
    }
    let mut tracker = remoteid::Tracker::new(remoteid::TrackerConfig {
        timeout: u64::MAX,
        ..Default::default()
    });
    let mut pacer = Pacer {
        first: None,
        on: a.realtime,
    };
    let mut n = 0;
    for (lt, ts, pkt) in read_pcap(&bytes)? {
        let frame = match lt {
            127 => match remoteid::wifi::strip_radiotap(pkt) {
                Ok(f) => f,
                Err(_) => continue,
            },
            105 => pkt,
            _ => continue,
        };
        let Ok(r) = remoteid::wifi::parse_frame(frame) else {
            continue;
        };
        let mac: String = r.source.iter().map(|b| format!("{b:02X}")).collect();
        let uas = tracker.ingest_wifi(&r, ts / 1000);
        pacer.wait(ts);
        for e in rid::tracked(uas, Some(&mac), ts, &opts) {
            a.sink.send(&e)?;
            n += 1;
        }
    }
    Ok(n)
}

fn main() {
    let r = parse_args().and_then(|mut a| match a.mode.as_str() {
        "klv" => run_klv(&mut a),
        "tlog" => run_tlog(&mut a),
        "pcap" => run_pcap(&mut a),
        "mavlink-udp" => run_mavlink_udp(&mut a),
        _ => usage(),
    });
    match r {
        Ok(n) => eprintln!("uas2cot: {n} events"),
        Err(e) => {
            eprintln!("uas2cot: {e}");
            std::process::exit(1);
        }
    }
}
