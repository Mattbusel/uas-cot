//! mavlinkm-check: check MAVLink-M traffic against military.xml.
//!
//! ```text
//! mavlinkm-check tlog <file.tlog>     a MAVLink telemetry log
//! mavlinkm-check raw  <file.bin|->    back-to-back MAVLink frames (a serial capture, or stdin)
//! mavlinkm-check udp  <bind addr>     live MAVLink over UDP, for example 0.0.0.0:14550 (Ctrl+C to stop)
//!
//! options:
//!   --dialect <military.xml>  check against this definition (its <include>s are loaded from
//!                             the same folder) instead of the embedded copy
//!   --all                     also print messages that pass
//!   --count <n>               udp: stop after n MAVLink-M messages
//! ```
//!
//! Only MAVLink-M messages (ids 53000 to 53999) are checked. Every one gets
//! the frame checks (id, length, checksum). Field checks (enums, bitmasks,
//! ranges) run only on in-scope messages; target, track, handover, fires,
//! munition, arming, weapon and engagement messages are never decoded and
//! their fields are never printed (see `uas_cot::mavlink_m::scope`). The
//! exit code is 1 if any message has an error, 2 for usage or input errors.

use std::collections::BTreeMap;
use std::io::{self, Read};
use uas_cot::mavlink_m::{self, validate, Dialect};

fn usage() -> ! {
    eprintln!(
        "usage: mavlinkm-check <tlog|raw|udp> <input> [--dialect military.xml] [--all] [--count n]"
    );
    std::process::exit(2)
}

#[derive(Default)]
struct Tally {
    per_msg: BTreeMap<String, (usize, usize)>,
    checked: usize,
    failed: usize,
    all: bool,
}

impl Tally {
    fn frame(&mut self, d: &Dialect, bytes: &[u8]) {
        let id = match bytes.first() {
            Some(0xFD) if bytes.len() >= 10 => {
                u32::from_le_bytes([bytes[7], bytes[8], bytes[9], 0])
            }
            Some(0xFE) if bytes.len() >= 6 => bytes[5] as u32,
            _ => return,
        };
        if !(53_000..54_000).contains(&id) {
            return;
        }
        let r = validate::check_frame(d, bytes);
        self.checked += 1;
        let slot = self
            .per_msg
            .entry(r.name.clone().unwrap_or_else(|| format!("id {id}")))
            .or_default();
        slot.0 += 1;
        if !r.is_ok() {
            self.failed += 1;
            slot.1 += 1;
        }
        if !r.is_ok() || self.all || !r.findings.is_empty() {
            println!("{r}");
        }
    }

    fn stream(&mut self, d: &Dialect, mut data: &[u8]) {
        while !data.is_empty() {
            match data.iter().position(|&b| b == 0xFD || b == 0xFE) {
                Some(0) => {}
                Some(i) => {
                    data = &data[i..];
                    continue;
                }
                None => return,
            }
            match validate::frame_len(data) {
                Some(n) => {
                    self.frame(d, &data[..n]);
                    data = &data[n..];
                }
                None => {
                    // truncated or bad header: resync one byte on
                    data = &data[1..];
                }
            }
        }
    }
}

fn main() {
    let mut args = std::env::args().skip(1);
    let mode = args.next().unwrap_or_else(|| usage());
    let input = args.next().unwrap_or_else(|| usage());
    let mut dialect_path = None;
    let mut all = false;
    let mut count = usize::MAX;
    while let Some(f) = args.next() {
        match f.as_str() {
            "--dialect" => dialect_path = Some(args.next().unwrap_or_else(|| usage())),
            "--all" => all = true,
            "--count" => {
                count = args
                    .next()
                    .and_then(|v| v.parse().ok())
                    .unwrap_or_else(|| usage())
            }
            _ => usage(),
        }
    }
    let owned;
    let d: &Dialect = match dialect_path {
        Some(p) => match Dialect::from_file(&p) {
            Ok(x) => {
                owned = x;
                &owned
            }
            Err(e) => {
                eprintln!("mavlinkm-check: {e}");
                std::process::exit(2)
            }
        },
        None => mavlink_m::military(),
    };
    let mut t = Tally {
        all,
        ..Default::default()
    };
    let r: io::Result<()> = (|| {
        match mode.as_str() {
            "tlog" => {
                let file = std::fs::File::open(&input)?;
                let mut reader = mavlink_tlog::TlogReader::new(io::BufReader::new(file));
                while let Some(rec) = reader.next_record()? {
                    t.frame(d, rec.frame.as_bytes());
                }
            }
            "raw" => {
                let mut bytes = Vec::new();
                if input == "-" {
                    io::stdin().read_to_end(&mut bytes)?;
                } else {
                    bytes = std::fs::read(&input)?;
                }
                t.stream(d, &bytes);
            }
            "udp" => {
                let sock = std::net::UdpSocket::bind(&input)?;
                let mut buf = [0u8; 65_536];
                while t.checked < count {
                    let n = sock.recv(&mut buf)?;
                    t.stream(d, &buf[..n]);
                }
            }
            _ => usage(),
        }
        Ok(())
    })();
    if let Err(e) = r {
        eprintln!("mavlinkm-check: {e}");
        std::process::exit(2);
    }
    eprintln!(
        "mavlinkm-check: {} MAVLink-M messages checked against {}{}, {} with errors",
        t.checked,
        d.name,
        if dialect_path_is_embedded(d) {
            format!(
                " (embedded, commit {})",
                &mavlink_m::MILITARY_XML_COMMIT[..12]
            )
        } else {
            String::new()
        },
        t.failed
    );
    for (name, (n, bad)) in &t.per_msg {
        eprintln!("  {name}: {n} checked, {bad} with errors");
    }
    if t.failed > 0 {
        std::process::exit(1);
    }
}

fn dialect_path_is_embedded(d: &Dialect) -> bool {
    std::ptr::eq(d, mavlink_m::military())
}
