//! Runs the `uas2cot` binary on public sample data and checks its output.
//!
//! * `day_flight.klv`, `night_flight_ir.klv`: KLV streams of the public
//!   "Day Flight.mpg" / "Night Flight IR.mpg" STANAG 4609 samples
//!   (samples.ffmpeg.org/MPEG2/mpegts-klv).
//! * `odid_wifi_bcn_sample.pcap`, `odid_wifi_sample.pcap`: Remote ID Wi-Fi
//!   Beacon and NAN captures from opendroneid/wireshark-dissector (Apache-2.0).
//! * `jjshoots_esp8266_beacon_1drone.pcap`: one fake drone from the
//!   jjshoots/RemoteIDSpoofer ESP8266 sketch, from the `remoteid` crate's
//!   spoof corpus (`remoteid/docs/spoof-corpus.md`).
//! * A MAVLink `.tlog` built here from known values.
//!
//! `tests/data/expected/*.cot` hold the expected events. CI validates them
//! against the MITRE CoT schemas and converts them to TAK Protocol v1 with
//! takproto (`tools/validate_cot.py`, `tools/takproto_check.py`).
//! `UAS_COT_REGENERATE=1 cargo test` rewrites them.

use std::path::{Path, PathBuf};
use std::process::Command;

fn data(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join(name)
}

fn run(args: &[&str]) -> String {
    let out = Command::new(env!("CARGO_BIN_EXE_uas2cot"))
        .args(args)
        .output()
        .unwrap();
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    String::from_utf8(out.stdout).unwrap()
}

fn check_golden(name: &str, got: &str) {
    let path = data("expected").join(name);
    if std::env::var_os("UAS_COT_REGENERATE").is_some() {
        std::fs::write(&path, got).unwrap();
    }
    let want = std::fs::read_to_string(&path)
        .unwrap()
        .replace("\r\n", "\n");
    assert_eq!(got, want, "{name} differs from the expected output");
}

/// Minimal well-formedness check: balanced tags, quoted attributes, one root.
fn well_formed(xml: &str) {
    let body = xml
        .strip_prefix("<?xml version=\"1.0\" encoding=\"UTF-8\"?>")
        .expect("declaration");
    let mut stack: Vec<&str> = Vec::new();
    let mut rest = body;
    while let Some(i) = rest.find('<') {
        assert!(
            !rest[..i].contains('&') || rest[..i].contains(';'),
            "bare & in text: {xml}"
        );
        rest = &rest[i + 1..];
        let end = rest.find('>').expect("unclosed tag");
        let tag = &rest[..end];
        rest = &rest[end + 1..];
        if let Some(name) = tag.strip_prefix('/') {
            assert_eq!(stack.pop(), Some(name), "mismatched close in {xml}");
        } else {
            let self_closing = tag.ends_with('/');
            let name = tag.split([' ', '/']).next().unwrap();
            assert_eq!(tag.matches('"').count() % 2, 0, "unbalanced quotes: {tag}");
            if !self_closing {
                stack.push(name);
            }
        }
        if stack.is_empty() {
            assert!(rest.is_empty(), "trailing data after root: {xml}");
        }
    }
    assert!(stack.is_empty(), "unclosed elements {stack:?}");
}

#[test]
fn klv_day_flight() {
    let out = run(&["klv", data("day_flight.klv").to_str().unwrap()]);
    let lines: Vec<&str> = out.lines().collect();
    assert_eq!(lines.len(), 12, "platform + SPI for each of the 6 packets");
    for l in &lines {
        well_formed(l);
    }
    // values come straight from the misb crate (itself checked against klvdata)
    assert!(lines[0].contains("uid=\"fmv-UAS\" type=\"a-f-A-M-F-Q\""));
    assert!(lines[0].contains("<point lat=\"54.6813233\" lon=\"-110.1685598\" hae=\"1532.3\""));
    assert!(
        lines[0].contains("<sensor azimuth=\"132.21\""),
        "heading 86.11 + relative azimuth 46.10"
    );
    assert!(lines[1].contains("type=\"b-m-p-s-p-i\""));
    assert!(lines[1].contains("<point lat=\"54.7491235\" lon=\"-110.0466381\""));
    assert!(lines[1].contains("<link uid=\"fmv-UAS\" type=\"a-f-A-M-F-Q\" relation=\"p-p\"/>"));
    check_golden("day_flight.cot", &out);
}

#[test]
fn klv_night_flight_with_options() {
    let out = run(&[
        "klv",
        data("night_flight_ir.klv").to_str().unwrap(),
        "--uid",
        "ir-1",
        "--video",
        "rtsp://10.0.0.5:8554/ir?a=1&b=2",
        "--affiliation",
        "n",
    ]);
    assert_eq!(out.lines().count(), 36);
    let first = out.lines().next().unwrap();
    well_formed(first);
    assert!(first.contains("uid=\"ir-1\" type=\"a-n-A-M-F-Q\""));
    assert!(first.contains("<__video url=\"rtsp://10.0.0.5:8554/ir?a=1&amp;b=2\"/>"));
    check_golden("night_flight_ir.cot", &out);
}

#[test]
fn remote_id_wifi_beacon() {
    let out = run(&["pcap", data("odid_wifi_bcn_sample.pcap").to_str().unwrap()]);
    let lines: Vec<&str> = out.lines().collect();
    assert!(!lines.is_empty());
    for l in &lines {
        well_formed(l);
    }
    // The values the remoteid README shows for this capture.
    let air = lines
        .iter()
        .rev()
        .find(|l| l.contains("uid=\"rid-MFG1A0123456789\""))
        .unwrap();
    assert!(air.contains("<point lat=\"45.5470818\" lon=\"-122.9668346\" hae=\"237.0\""));
    assert!(air.contains("<track course=\"280.0\" speed=\"20.50\"/>"));
    assert!(air.contains("Operator ID: GBR-OP-123ABCD."));
    // A real transmitter: remoteid finds nothing that contradicts it.
    assert!(air.contains("Trust: plausible.</remarks>"));
    // The capture's System message says its ground position is the takeoff
    // point, so it is a takeoff marker, not an operator marker.
    assert!(!out.contains("uid=\"rid-op-MFG1A0123456789\""));
    let home = lines
        .iter()
        .rev()
        .find(|l| l.contains("uid=\"rid-takeoff-MFG1A0123456789\""))
        .unwrap();
    assert!(home.contains("<point lat=\"45.5443876\" lon=\"-122.9726866\""));
    assert!(home.contains("type=\"b-m-p-s-m\""));
    assert!(home.contains(
        "<contact callsign=\"Takeoff point of MFG1A0123456789 (operator GBR-OP-123ABCD)\"/>"
    ));
    check_golden("odid_wifi_bcn_sample.cot", &out);
}

#[test]
fn remote_id_wifi_nan() {
    let out = run(&["pcap", data("odid_wifi_sample.pcap").to_str().unwrap()]);
    assert!(out.lines().count() > 0);
    for l in out.lines() {
        well_formed(l);
    }
    check_golden("odid_wifi_sample.cot", &out);
}

#[test]
fn remote_id_affiliation_rules() {
    let path = data("odid_wifi_bcn_sample.pcap");
    let p = path.to_str().unwrap();
    let plain = run(&["pcap", p]);
    assert!(plain.contains("uid=\"rid-MFG1A0123456789\" type=\"a-u-A-M-F-Q\""));
    let ours = run(&["pcap", p, "--friendly", "MFG1A0123456789"]);
    let air: Vec<&str> = ours
        .lines()
        .filter(|l| l.contains("uid=\"rid-MFG1A0123456789\""))
        .collect();
    assert!(!air.is_empty());
    for l in &air {
        assert!(l.contains("type=\"a-f-A-M-F-Q\""), "{l}");
    }
    // By Operator ID too.
    let ours = run(&["pcap", p, "--friendly", "GBR-OP-123ABCD"]);
    assert!(ours.contains("uid=\"rid-MFG1A0123456789\" type=\"a-f-A-M-F-Q\""));
}

#[test]
fn spoofer_with_a_friendly_id_is_not_drawn_friendly() {
    let path = data("jjshoots_esp8266_beacon_1drone.pcap");
    let p = path.to_str().unwrap();
    let plain = run(&["pcap", p]);
    let id = plain
        .lines()
        .find_map(|l| {
            let rest = l.split("<contact callsign=\"").nth(1)?;
            l.contains("type=\"a-")
                .then(|| rest.split('"').next().unwrap().to_string())
        })
        .unwrap();
    let out = run(&["pcap", p, "--friendly", &id, "--suspicious-as", "s"]);
    let last = out.lines().rev().find(|l| l.contains("type=\"a-")).unwrap();
    assert!(last.contains("Trust: suspicious"), "{last}");
    assert!(last.contains("type=\"a-s-A"), "{last}");
}

#[test]
fn remote_id_spoofer_is_flagged_in_remarks() {
    let path = data("jjshoots_esp8266_beacon_1drone.pcap");
    let out = run(&["pcap", path.to_str().unwrap()]);
    let air: Vec<&str> = out.lines().filter(|l| l.contains("type=\"a-")).collect();
    assert!(!air.is_empty());
    for l in &air {
        well_formed(l);
        assert!(
            l.contains(" Trust: "),
            "every aircraft event carries a verdict"
        );
    }
    // The first frames have nothing to compare against; once the fake
    // course and position disagree, the marker says so.
    let last = air.last().unwrap();
    assert!(
        last.contains("Trust: suspicious (reported_course_mismatch"),
        "{last}"
    );
}

fn synthetic_tlog(path: &Path) {
    use mavlink_tlog::frame::{encode, Header, Version};
    let mut w = mavlink_tlog::TlogWriter::new(std::fs::File::create(path).unwrap());
    let mut buf = [0u8; 300];
    let t0 = 1_700_000_000_000_000u64;
    for i in 0..10u32 {
        // HEARTBEAT: custom_mode, type = 2 (quadrotor), autopilot 3, base_mode, status, version 3
        let hb = [0, 0, 0, 0, 2, 3, 0x51, 4, 3];
        let n = encode(
            Version::V2,
            Header {
                seq: (2 * i) as u8,
                sys_id: 7,
                comp_id: 1,
                msg_id: 0,
            },
            &hb,
            None,
            &mut buf,
        )
        .unwrap();
        w.write_frame(t0 + i as u64 * 1_000_000, &buf[..n]).unwrap();
        let mut p = Vec::new();
        p.extend_from_slice(&(i * 1000).to_le_bytes());
        p.extend_from_slice(&(473_977_418i32 + i as i32 * 100).to_le_bytes());
        p.extend_from_slice(&85_455_939i32.to_le_bytes());
        p.extend_from_slice(&488_500i32.to_le_bytes());
        p.extend_from_slice(&20_000i32.to_le_bytes());
        p.extend_from_slice(&300i16.to_le_bytes()); // 3 m/s north
        p.extend_from_slice(&400i16.to_le_bytes()); // 4 m/s east
        p.extend_from_slice(&0i16.to_le_bytes());
        p.extend_from_slice(&u16::MAX.to_le_bytes());
        let n = encode(
            Version::V2,
            Header {
                seq: (2 * i + 1) as u8,
                sys_id: 7,
                comp_id: 1,
                msg_id: 33,
            },
            &p,
            None,
            &mut buf,
        )
        .unwrap();
        w.write_frame(t0 + i as u64 * 1_000_000 + 500, &buf[..n])
            .unwrap();
    }
    w.flush().unwrap();
}

#[test]
fn mavlink_tlog() {
    let dir = std::env::temp_dir().join(format!("uas-cot-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("synthetic.tlog");
    synthetic_tlog(&path);
    let out = run(&["tlog", path.to_str().unwrap()]);
    let lines: Vec<&str> = out.lines().collect();
    assert_eq!(lines.len(), 10);
    for l in &lines {
        well_formed(l);
    }
    assert!(lines[0].contains("uid=\"mav-7\" type=\"a-f-A-M-F-Q\""));
    assert!(lines[0].contains("time=\"2023-11-14T22:13:20.000Z\""));
    assert!(lines[0].contains("<point lat=\"47.3977418\" lon=\"8.5455939\" hae=\"488.5\""));
    // course from velocity (heading unknown): atan2(4, 3) = 53.13 degrees, 5 m/s
    assert!(
        lines[0].contains("<track course=\"53.1\" speed=\"5.00\"/>"),
        "{}",
        lines[0]
    );
    assert!(lines[9].contains("lat=\"47.3978318\""));
    check_golden("synthetic.cot", &out);
    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn sends_over_udp() {
    let rx = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
    rx.set_read_timeout(Some(std::time::Duration::from_secs(10)))
        .unwrap();
    let addr = rx.local_addr().unwrap().to_string();
    let out = Command::new(env!("CARGO_BIN_EXE_uas2cot"))
        .args([
            "klv",
            data("day_flight.klv").to_str().unwrap(),
            "--udp",
            &addr,
        ])
        .output()
        .unwrap();
    assert!(out.status.success());
    assert!(out.stdout.is_empty());
    let mut buf = [0u8; 4096];
    for _ in 0..12 {
        let n = rx.recv(&mut buf).unwrap();
        well_formed(std::str::from_utf8(&buf[..n]).unwrap());
    }
}
