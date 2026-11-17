//! MAVLink-M: layout against pymavlink, CoT types against MIL-STD-2525B,
//! the gateway end to end through `uas2cot`, and the conformance checker.
#![cfg(feature = "mavlink-m")]

use std::path::{Path, PathBuf};
use std::process::Command;
use uas_cot::mavlink_m::{self, validate, Gateway, Value};

fn data(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join(name)
}

const TRACK: [u8; 16] = [
    0x5e, 0x1d, 0x0c, 0x41, 0x9a, 0x2b, 0x4c, 0x8e, 0x91, 0x3f, 0x60, 0x7d, 0x22, 0xa4, 0x18, 0xb3,
];

fn uid_value() -> Value {
    Value::Array(TRACK.iter().map(|&b| Value::Uint(b as u64)).collect())
}

fn payload(name: &str, values: &[(&str, Value)]) -> (u32, Vec<u8>) {
    let m = mavlink_m::military().message_by_name(name).unwrap();
    (m.id, m.encode(values).unwrap())
}

fn frame(sys: u8, id: u32, p: &[u8]) -> Vec<u8> {
    let extra = mavlink_m::crc_extra(id).unwrap();
    let mut buf = [0u8; 300];
    let n = mavlink_tlog::encode(
        mavlink_tlog::Version::V2,
        mavlink_tlog::Header {
            seq: 0,
            sys_id: sys,
            comp_id: 1,
            msg_id: id,
        },
        p,
        Some(extra),
        &mut buf,
    )
    .unwrap();
    buf[..n].to_vec()
}

fn handover() -> (u32, Vec<u8>) {
    payload(
        "TARGET_HANDOVER",
        &[
            ("time_usec", Value::Uint(1_763_400_000_000_000)),
            ("target_system", Value::Uint(2)),
            ("valid_until_usec", Value::Uint(1_763_400_010_000_000)),
            ("lat", Value::Int(473_941_489)),
            ("lon", Value::Int(85_429_382)),
            ("alt", Value::Float(488.0)),
            ("vx", Value::Float(4.8)),
            ("vy", Value::Float(6.4)),
            ("cov_pos_x", Value::Float(25.0)),
            ("cov_pos_y", Value::Float(16.0)),
            ("cov_pos_z", Value::Float(9.0)),
            ("target_name", Value::Text("SIM TRUCK".into())),
            ("confidence_score", Value::Float(0.72)),
            ("authorization", Value::Array(vec![Value::Uint(0xAB); 8])),
            ("target_class", Value::Uint(5)),
            ("track_uid", uid_value()),
        ],
    )
}

fn identity(stanag: u64, context: u64) -> (u32, Vec<u8>) {
    payload(
        "TRACK_IDENTITY",
        &[
            ("time_usec", Value::Uint(1_763_400_000_000_000)),
            ("track_uid", uid_value()),
            ("id_confidence", Value::Float(0.72)),
            ("origin_sysid", Value::Uint(1)),
            ("id_method", Value::Uint(1)),
            ("pid_status", Value::Uint(1)),
            ("target_class", Value::Uint(5)),
            ("stanag_identity", Value::Uint(stanag)),
            ("environment", Value::Uint(5)),
            ("atr_confidence_pct", Value::Uint(255)),
            ("sidc_context", Value::Uint(context)),
            ("external_track_number", Value::Text("TN 04521".into())),
            ("external_track_type", Value::Uint(1)),
        ],
    )
}

#[test]
fn layout_matches_pymavlink() {
    let d = mavlink_m::military();
    let reference = std::fs::read_to_string(data("mavlink_m_reference.txt")).unwrap();
    let mut n = 0;
    for line in reference
        .lines()
        .filter(|l| !l.starts_with('#') && !l.is_empty())
    {
        let f: Vec<&str> = line.split_whitespace().collect();
        let id: u32 = f[0].parse().unwrap();
        let m = d.message(id).unwrap_or_else(|| panic!("{} missing", f[1]));
        assert_eq!(m.name, f[1]);
        assert_eq!(m.crc_extra.to_string(), f[2], "{} CRC_EXTRA", m.name);
        assert_eq!(m.base_len.to_string(), f[3], "{} base length", m.name);
        assert_eq!(
            m.max_len.to_string(),
            f[4],
            "{} length with extensions",
            m.name
        );
        n += 1;
    }
    assert_eq!(n, 35);
    assert_eq!(n, d.messages.len());
}

#[test]
fn every_cot_type_is_a_2525b_function() {
    let table = std::fs::read_to_string(data("2525b_function_ids.txt")).unwrap();
    let known: std::collections::HashSet<&str> =
        table.lines().filter(|l| !l.starts_with('#')).collect();
    let as_code = |t: &str| {
        let mut parts = t.split('-');
        let dim = parts.next().unwrap();
        let func: String = parts.collect();
        format!("{dim} {func:-<6}")
    };
    let mut tails: Vec<&str> = Vec::new();
    for c in 0..=255u8 {
        tails.extend(mavlink_m::class_type(c, false));
        tails.extend(mavlink_m::class_type(c, true));
        tails.extend(mavlink_m::domain_dimension(c));
        tails.extend(mavlink_m::environment_dimension(c));
        tails.extend(mavlink_m::ppli_dimension(c));
    }
    for t in tails {
        assert!(
            known.contains(as_code(t).as_str()),
            "{t} is not a 2525B function ID"
        );
    }
    // every defined target class with a fixed dimension has a mapping
    let classes = &mavlink_m::military().enums["MAVLINK_M_TARGET_CLASS"];
    let unmapped: Vec<&str> = classes
        .entries
        .iter()
        .filter(|e| mavlink_m::class_type(e.value as u8, false).is_none())
        .map(|e| e.name.as_str())
        .collect();
    assert_eq!(
        unmapped,
        [
            "MAVLINK_M_TARGET_CLASS_UNKNOWN",
            "MAVLINK_M_TARGET_CLASS_EMITTER_UNKNOWN",
            "MAVLINK_M_TARGET_CLASS_RADAR_EMITTER",
            "MAVLINK_M_TARGET_CLASS_JAMMER",
            "MAVLINK_M_TARGET_CLASS_COMMUNICATIONS_NODE",
            "MAVLINK_M_TARGET_CLASS_NETWORK_NODE",
            "MAVLINK_M_TARGET_CLASS_OTHER",
        ]
    );
}

#[test]
fn handover_joins_identity_by_track_uid() {
    let mut gw = Gateway::default();
    let now = 1_763_400_001_000_000;
    // kinematics first, identity unknown: affiliation from target_force (unknown)
    let (hid, hp) = handover();
    let e = gw.ingest(1, hid, &hp, now).unwrap();
    assert_eq!(
        e.uid,
        "mavlink-m-track-5e1d0c41-9a2b-4c8e-913f-607d22a418b3"
    );
    assert_eq!(e.kind, "a-u-G-E-V-U");
    assert!((e.point.lat - 47.3941489).abs() < 1e-9);
    assert_eq!(e.point.ce, 5.0);
    assert_eq!(e.point.le, 3.0);
    let t = e.track.unwrap();
    assert!((t.speed - 8.0).abs() < 1e-6 && (t.course - 53.130102).abs() < 1e-4);
    assert_eq!(e.stale_us, 1_763_400_010_000_000);
    assert_eq!(e.links[0].uid, "mav-1");
    // identity arrives: the same marker is re-issued as suspect, flagged simulation
    let (iid, ip) = identity(5, 2);
    let e2 = gw.ingest(1, iid, &ip, now).unwrap();
    assert_eq!(e2.uid, e.uid);
    assert_eq!(e2.kind, "a-s-G-E-V-U");
    assert_eq!(e2.callsign.as_deref(), Some("SIM TRUCK"));
    let r = e2.remarks.clone().unwrap();
    assert!(r.contains("SIMULATION track."), "{r}");
    assert!(r.contains("Identity: suspect."), "{r}");
    assert!(r.contains("External track TN 04521 (link16)."), "{r}");
    // later kinematics keep the identity
    let e3 = gw.ingest(1, hid, &hp, now + 1_000_000).unwrap();
    assert_eq!(e3.kind, "a-s-G-E-V-U");
    // engagement fields are not carried
    let xml = e3.to_xml();
    assert!(!xml.to_lowercase().contains("authoriz"), "{xml}");
    assert!(!xml.contains("ABAB"), "{xml}");
}

#[test]
fn target_excludes_engagement_fields() {
    let (id, p) = payload(
        "TARGET",
        &[
            ("time_usec", Value::Uint(1_763_400_000_000_000)),
            ("target_id", Value::Uint(17)),
            ("target_name", Value::Text("BRIDGE 3".into())),
            ("lat", Value::Int(473_977_418)),
            ("lon", Value::Int(85_455_939)),
            ("alt", Value::Float(470.0)),
            ("cep_desired", Value::Float(3.0)),
            ("flags", Value::Uint(1 | 2 | 32)),
            ("package_endpoint_ip_1", Value::Uint(0xC0A8010A)),
            ("package_endpoint_port", Value::Uint(8443)),
            ("package_path", Value::Text("pkg/secret-path".into())),
            ("prf_code", Value::Uint(1688)),
            ("target_class", Value::Uint(191)),
            ("target_domain", Value::Uint(8)),
            ("target_force", Value::Uint(10)),
            ("confidence", Value::Uint(8_500)),
            ("tle_category", Value::Uint(1)),
            ("restricted_target_flags", Value::Uint(1)),
        ],
    );
    let mut gw = Gateway::default();
    let e = gw.ingest(4, id, &p, 1_763_400_000_500_000).unwrap();
    assert_eq!(e.uid, "mavlink-m-target-4-17");
    assert_eq!(e.kind, "a-n-G-I-T");
    let xml = e.to_xml();
    for banned in [
        "1688",
        "secret-path",
        "8443",
        "192.168",
        "cep",
        "authorization required",
        "restricted",
    ] {
        assert!(
            !xml.to_lowercase().contains(banned),
            "{banned} leaked: {xml}"
        );
    }
    let r = e.remarks.unwrap();
    assert!(r.contains("Confidence 0.85."), "{r}");
    assert!(r.contains("Flags: gnss degraded, moving."), "{r}");
}

#[test]
fn participant_position_is_a_friendly_marker() {
    let (id, p) = payload(
        "PARTICIPANT_POSITION",
        &[
            ("lat", Value::Int(473_977_418)),
            ("lon", Value::Int(85_455_939)),
            ("alt", Value::Float(f64::NAN)),
            ("vx", Value::Float(0.0)),
            ("vy", Value::Float(-12.0)),
            ("course", Value::Float(f64::NAN)),
            ("callsign", Value::Text("HAWK 2".into())),
            ("origin_sysid", Value::Uint(2)),
            ("stanag_identity", Value::Uint(3)),
            ("ppli_type", Value::Uint(1)),
        ],
    );
    let e = Gateway::default().ingest(2, id, &p, 1).unwrap();
    assert_eq!(
        (e.uid.as_str(), e.kind.as_str()),
        ("mavlink-m-ppli-2", "a-f-A")
    );
    assert_eq!(e.point.hae, 0.0);
    assert_eq!(e.track.unwrap().course, 270.0);
    assert!(e.to_xml().contains("callsign=\"HAWK 2\""));
}

#[test]
fn checker_accepts_good_frames() {
    let d = mavlink_m::military();
    for (id, p) in [handover(), identity(6, 0)] {
        let f = frame(1, id, &p);
        let r = validate::check_frame(d, &f);
        assert!(r.is_ok() && r.findings.is_empty(), "{r}");
        assert_eq!(r.sys_id, Some(1));
    }
}

#[test]
fn checker_finds_errors() {
    let d = mavlink_m::military();
    let errors = |r: &validate::Report| -> Vec<String> {
        r.findings
            .iter()
            .filter(|f| f.severity == validate::Severity::Error)
            .map(|f| format!("{}: {}", f.field.clone().unwrap_or_default(), f.message))
            .collect()
    };

    // undefined enum value
    let (id, mut p) = identity(5, 2);
    let off = d.message(id).unwrap().field("pid_status").unwrap().offset;
    p[off] = 9;
    let r = validate::check_payload(d, id, &p, true);
    assert_eq!(
        errors(&r),
        ["pid_status: 9 is not a defined MAVLINK_M_PID_STATUS value"]
    );

    // a protected (extended-Hamming) enum reports corruption and the nearest codeword
    let st = d.message_by_name("PAYLOAD_STATE").unwrap();
    let f = st.field("health").unwrap();
    let mut p2 = vec![0u8; st.max_len];
    p2[f.offset] = 14; // one bit away from PAYLOAD_HEALTH_NOMINAL (15)
    let r = validate::check_payload(d, st.id, &p2, true);
    let e = errors(&r).join(
        "
",
    );
    assert!(
        e.contains("health: 14 is not a defined PAYLOAD_HEALTH value: this enum's values are codewords at least 4 bits apart")
            && e.contains("nearest codeword PAYLOAD_HEALTH_NOMINAL, 1 bits away"),
        "{e}"
    );

    // "Never 0": a handover addressed to every system
    let (hid, mut hp) = handover();
    let off = d
        .message(hid)
        .unwrap()
        .field("target_system")
        .unwrap()
        .offset;
    hp[off] = 0;
    let r = validate::check_payload(d, hid, &hp, true);
    assert_eq!(
        errors(&r),
        ["target_system: is 0; the definition says it is never 0"]
    );

    // out-of-range latitude and confidence
    let (hid, mut hp) = handover();
    let m = d.message(hid).unwrap();
    let lat = m.field("lat").unwrap().offset;
    hp[lat..lat + 4].copy_from_slice(&950_000_000i32.to_le_bytes());
    let c = m.field("confidence_score").unwrap().offset;
    hp[c..c + 4].copy_from_slice(&1.5f32.to_le_bytes());
    let r = validate::check_payload(d, hid, &hp, true);
    assert_eq!(
        errors(&r),
        [
            "lat: 950000000 degE7 is out of range (|value| <= 900000000)",
            "confidence_score: 1.5 is outside [0.0-1.0]"
        ]
    );

    // undefined bitmask bits
    let tgt = d.message_by_name("TARGET").unwrap();
    let mut tp = vec![0u8; tgt.max_len];
    let fo = tgt.field("flags").unwrap().offset;
    tp[fo..fo + 4].copy_from_slice(&0x1000u32.to_le_bytes());
    let r = validate::check_payload(d, tgt.id, &tp, true);
    assert_eq!(
        errors(&r),
        ["flags: bits 0x1000 are not defined in bitmask MAVLINK_M_TARGET_FLAGS"]
    );

    // wrong checksum: a sender built from another definition
    let (hid, hp) = handover();
    let mut f = frame(1, hid, &hp);
    let n = f.len();
    f[n - 1] ^= 0xFF;
    let r = validate::check_frame(d, &f);
    assert!(errors(&r)[0].contains("does not match"), "{r}");

    // unknown id, too long, MAVLink 1
    assert!(!validate::check_payload(d, 53_999, &[1], true).is_ok());
    assert!(!validate::check_payload(d, hid, &vec![1u8; 300], true).is_ok());
    assert!(!validate::check_payload(d, hid, &hp, false).is_ok());
}

#[test]
fn from_file_resolves_includes() {
    let dir = std::env::temp_dir().join(format!("uas-cot-mlm-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join("military.xml"), mavlink_m::MILITARY_XML).unwrap();
    std::fs::write(
        dir.join("common.xml"),
        r#"<mavlink><enums><enum name="MAV_CMD"><entry value="16" name="MAV_CMD_NAV_WAYPOINT"/></enum></enums>
           <messages><message id="0" name="HEARTBEAT"><field type="uint8_t" name="type">t</field></message></messages></mavlink>"#,
    )
    .unwrap();
    let d = uas_cot::mavlink_m::Dialect::from_file(dir.join("military.xml")).unwrap();
    assert!(
        d.unresolved_includes.is_empty(),
        "{:?}",
        d.unresolved_includes
    );
    assert!(d.message(0).is_some() && d.message(53002).is_some());
    let cmds = &d.enums["MAV_CMD"].entries;
    assert_eq!(cmds[0].name, "MAV_CMD_NAV_WAYPOINT");
    assert!(cmds.iter().any(|e| e.value == 53091));
    assert_eq!(mavlink_m::military().unresolved_includes, ["common.xml"]);
    std::fs::remove_dir_all(dir).ok();
}

/// Builds a .tlog with vehicle telemetry and MAVLink-M traffic, runs
/// `uas2cot tlog` and `mavlinkm-check tlog` on it.
#[test]
fn uas2cot_and_checker_on_a_tlog() {
    let path = std::env::temp_dir().join(format!("uas-cot-mlm-{}.tlog", std::process::id()));
    {
        let mut w = mavlink_tlog::TlogWriter::new(std::fs::File::create(&path).unwrap());
        let t0 = 1_763_400_000_000_000u64;
        // HEARTBEAT (quadrotor) and GLOBAL_POSITION_INT from system 1
        let mut hb = vec![0u8; 4];
        hb.extend_from_slice(&[2, 12, 0x81, 4, 3]);
        let mut buf = [0u8; 300];
        let n = mavlink_tlog::encode(
            mavlink_tlog::Version::V2,
            mavlink_tlog::Header {
                seq: 0,
                sys_id: 1,
                comp_id: 1,
                msg_id: 0,
            },
            &hb,
            None,
            &mut buf,
        )
        .unwrap();
        w.write_frame(t0, &buf[..n]).unwrap();
        let mut gpi = Vec::new();
        gpi.extend_from_slice(&0u32.to_le_bytes());
        for v in [473_977_418i32, 85_455_939, 608_000, 120_000] {
            gpi.extend_from_slice(&v.to_le_bytes());
        }
        for v in [500i16, 0, 0] {
            gpi.extend_from_slice(&v.to_le_bytes());
        }
        gpi.extend_from_slice(&9000u16.to_le_bytes());
        let n = mavlink_tlog::encode(
            mavlink_tlog::Version::V2,
            mavlink_tlog::Header {
                seq: 1,
                sys_id: 1,
                comp_id: 1,
                msg_id: 33,
            },
            &gpi,
            None,
            &mut buf,
        )
        .unwrap();
        w.write_frame(t0, &buf[..n]).unwrap();
        let (hid, hp) = handover();
        w.write_frame(t0 + 100_000, &frame(1, hid, &hp)).unwrap();
        let (iid, ip) = identity(6, 0);
        w.write_frame(t0 + 200_000, &frame(1, iid, &ip)).unwrap();
        // a corrupt one: undefined pid_status
        let mut bad = ip.clone();
        let off = mavlink_m::military()
            .message(iid)
            .unwrap()
            .field("pid_status")
            .unwrap()
            .offset;
        bad[off] = 9;
        w.write_frame(t0 + 300_000, &frame(1, iid, &bad)).unwrap();
        w.flush().unwrap();
    }
    let p = path.to_str().unwrap();
    let out = Command::new(env!("CARGO_BIN_EXE_uas2cot"))
        .args(["tlog", p])
        .output()
        .unwrap();
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let cot = String::from_utf8(out.stdout).unwrap();
    let lines: Vec<&str> = cot.lines().collect();
    assert_eq!(
        lines.len(),
        3,
        "the corrupt TRACK_IDENTITY is dropped: {cot}"
    );
    assert!(
        lines[0].contains("uid=\"mav-1\" type=\"a-f-A-M-F-Q\""),
        "{}",
        lines[0]
    );
    assert!(lines[1].contains("type=\"a-u-G-E-V-U\""), "{}", lines[1]);
    assert!(lines[2].contains("type=\"a-h-G-E-V-U\""), "{}", lines[2]);
    assert!(
        lines[2].contains("<link uid=\"mav-1\" type=\"a-f-A\" relation=\"p-p\"/>"),
        "{}",
        lines[2]
    );

    let out = Command::new(env!("CARGO_BIN_EXE_mavlinkm-check"))
        .args(["tlog", p])
        .output()
        .unwrap();
    assert_eq!(out.status.code(), Some(1));
    let report = String::from_utf8(out.stdout).unwrap();
    assert!(report.contains("TRACK_IDENTITY (53000) from 1/1\n  error [pid_status]: 9 is not a defined MAVLINK_M_PID_STATUS value"), "{report}");
    let summary = String::from_utf8(out.stderr).unwrap();
    assert!(
        summary.contains("3 MAVLink-M messages checked against military"),
        "{summary}"
    );
    assert!(
        summary.contains("TRACK_IDENTITY: 2 checked, 1 with errors"),
        "{summary}"
    );
    std::fs::remove_file(path).ok();
}
