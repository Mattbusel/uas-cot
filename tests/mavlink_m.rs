//! MAVLink-M: layout against pymavlink, the scope block (no target, track,
//! fires or engagement message ever produces CoT or is decoded), friendly
//! position reports end to end through `uas2cot`, and the conformance
//! checker.
#![cfg(feature = "mavlink-m")]

use std::path::{Path, PathBuf};
use std::process::Command;
use uas_cot::mavlink_m::{self, scope, validate, FieldType, Gateway, Message, Options, Value};

fn data(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join(name)
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

fn participant(ppli_type: u64) -> (u32, Vec<u8>) {
    payload(
        "PARTICIPANT_POSITION",
        &[
            ("time_usec", Value::Uint(1_763_400_000_000_000)),
            ("lat", Value::Int(473_977_418)),
            ("lon", Value::Int(85_455_939)),
            ("alt", Value::Float(f64::NAN)),
            ("vx", Value::Float(0.0)),
            ("vy", Value::Float(-12.0)),
            ("course", Value::Float(f64::NAN)),
            ("callsign", Value::Text("HAWK 2".into())),
            ("origin_sysid", Value::Uint(2)),
            ("stanag_identity", Value::Uint(3)),
            ("ppli_type", Value::Uint(ppli_type)),
        ],
    )
}

/// Every message the scope rules must refuse, by name: fires, munition,
/// target, track, engagement and handover messages of `military.xml`, and
/// the generic payload messages, which can report payload arming and
/// activation state.
const MUST_BLOCK: &[&str] = &[
    "TRACK_IDENTITY",
    "TARGET_CUE",
    "TARGET_HANDOVER",
    "TARGET",
    "TARGET_SET_COORD",
    "TARGET_BOX_COORD",
    "TARGET_AUTHORIZATION",
    "FIRES",
    "SPLASH_CORRECTION",
    "BATTLE_DAMAGE_ASSESSMENT",
    "ENGAGEMENT_DIRECTIVE",
    "CALL_FOR_FIRE",
    "ESAD_STATE",
    "ESAD_ARMING",
    "ESAD_CONFIG",
    "STORE_STATUS",
    "STORE_SENSOR_STATUS",
    "STORE_MUNITION",
    "LOITER_MUNITION_CONTROL",
    "RWS_POSE",
    "RWS_STATE",
    "SENSOR_TASKING",
    "CAS_9LINE",
    "TERMINAL_CONTROL",
    "MAVLINK_M_ACK",
    "PAYLOAD_INFO",
    "PAYLOAD_STATE",
    "PAYLOAD_RESOURCE",
    "PAYLOAD_INHIBIT",
    "PAYLOAD_MEASUREMENT",
    "PAYLOAD_OPERATION_REQUEST",
    "PAYLOAD_OPERATION_STATUS",
    "PAYLOAD_CATALOGUE_STATUS",
    "PAYLOAD_OPERATION_CANCEL",
];

/// The reviewed in-scope messages: friendly positions only. Only
/// PARTICIPANT_POSITION becomes CoT.
const IN_SCOPE: &[&str] = &["PARTICIPANT_POSITION"];

fn put(p: &mut [u8], off: usize, ty: FieldType, x: f64) {
    let b: Vec<u8> = match ty {
        FieldType::Char | FieldType::U8 | FieldType::I8 => vec![x as i64 as u8],
        FieldType::U16 | FieldType::I16 => (x as i64 as u16).to_le_bytes().to_vec(),
        FieldType::U32 | FieldType::I32 => (x as i64 as u32).to_le_bytes().to_vec(),
        FieldType::U64 | FieldType::I64 => (x as i64 as u64).to_le_bytes().to_vec(),
        FieldType::F32 => (x as f32).to_le_bytes().to_vec(),
        FieldType::F64 => x.to_le_bytes().to_vec(),
    };
    p[off..off + b.len()].copy_from_slice(&b);
}

/// A full-length payload of `m` written byte by byte (not through
/// `Message::encode`, which refuses blocked messages) with values that
/// would pass the field checks: defined enum values, a real position,
/// in-range numbers, non-zero UIDs. If the block were missing, this is
/// what a gateway would translate.
fn plausible(m: &Message) -> Vec<u8> {
    let d = mavlink_m::military();
    let mut p = vec![0u8; m.max_len];
    for f in &m.fields {
        let n = f.array_len.unwrap_or(1);
        for k in 0..n {
            let off = f.offset + k * f.ty.size();
            let x = if let Some(e) = f.enum_name.as_deref().and_then(|e| d.enums.get(e)) {
                if e.bitmask {
                    0.0
                } else {
                    e.entries
                        .iter()
                        .map(|y| y.value)
                        .find(|&v| v != 0)
                        .unwrap_or(0) as f64
                }
            } else if f.units.as_deref() == Some("degE7") {
                if f.name.contains("lon") {
                    85_455_939.0
                } else {
                    473_977_418.0
                }
            } else if f.ty == FieldType::Char {
                if k == 0 {
                    b'X' as f64
                } else {
                    0.0
                }
            } else if f.description.contains("[0.0-1.0]") {
                0.5
            } else {
                1.0
            };
            put(&mut p, off, f.ty, x);
        }
    }
    p
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

/// Every message of the embedded dialect is classified: blocked, or on the
/// reviewed in-scope list. A new message in a dialect update fails here
/// until someone decides which side it is on.
#[test]
fn every_message_is_classified() {
    let d = mavlink_m::military();
    for name in MUST_BLOCK {
        let m = d
            .message_by_name(name)
            .unwrap_or_else(|| panic!("{name} missing"));
        assert!(m.is_blocked(), "{name} must be blocked");
        assert!(scope::is_blocked(m.id, name), "{name} must be blocked");
        assert!(!scope::is_translated(m.id), "{name} must not be translated");
    }
    for name in IN_SCOPE {
        let m = d.message_by_name(name).unwrap();
        assert!(!m.is_blocked(), "{name} is in scope");
    }
    for m in d.messages.values() {
        assert!(
            MUST_BLOCK.contains(&m.name.as_str()) || IN_SCOPE.contains(&m.name.as_str()),
            "{} is not classified",
            m.name
        );
        // the blocked list itself agrees with the dialect's ids
        if let Some(&(_, n)) = scope::BLOCKED.iter().find(|(id, _)| *id == m.id) {
            assert_eq!(n, m.name);
        }
    }
    assert_eq!(scope::BLOCKED.len(), MUST_BLOCK.len());
    assert_eq!(MUST_BLOCK.len() + IN_SCOPE.len(), d.messages.len());
    for name in MUST_BLOCK.iter().filter(|n| n.starts_with("PAYLOAD_")) {
        let m = d.message_by_name(name).unwrap();
        assert!(scope::is_blocked_id(m.id), "{name} must be on BLOCKED");
    }
    assert_eq!(scope::TRANSLATED, &[(53003, "PARTICIPANT_POSITION")]);
}

/// No blocked message produces CoT: zero-filled, all-ones, and a plausible
/// payload that would pass validation, with and without `reject_invalid`.
#[test]
fn blocked_messages_never_produce_cot() {
    let d = mavlink_m::military();
    let mut sent = 0;
    for reject_invalid in [true, false] {
        let mut gw = Gateway::new(Options {
            reject_invalid,
            ..Options::default()
        });
        let mut n = 0;
        for name in MUST_BLOCK {
            let m = d.message_by_name(name).unwrap();
            for p in [vec![0u8; m.max_len], vec![0xFFu8; m.max_len], plausible(m)] {
                for sys in [1u8, 7, 255] {
                    let e = gw.ingest(sys, m.id, &p, 1_763_400_000_000_000);
                    assert!(e.is_none(), "{name} produced CoT: {:?}", e);
                    n += 1;
                }
            }
        }
        assert_eq!(gw.blocked(), n);
        sent += n;
    }
    assert_eq!(sent, 2 * 34 * 3 * 3);
}

/// Of the in-scope messages, only PARTICIPANT_POSITION becomes CoT.
#[test]
fn only_participant_position_is_translated() {
    let d = mavlink_m::military();
    let mut gw = Gateway::new(Options {
        reject_invalid: false,
        ..Options::default()
    });
    for name in IN_SCOPE {
        let m = d.message_by_name(name).unwrap();
        let e = gw.ingest(1, m.id, &plausible(m), 1_763_400_000_000_000);
        assert_eq!(e.is_some(), *name == "PARTICIPANT_POSITION", "{name}");
    }
    assert_eq!(gw.blocked(), 0);
}

/// Blocked messages are not decoded or built by the crate's own API.
#[test]
fn blocked_messages_are_not_decoded_or_encoded() {
    let d = mavlink_m::military();
    for name in MUST_BLOCK {
        let m = d.message_by_name(name).unwrap();
        let p = plausible(m);
        assert!(m.decode(&p).is_empty(), "{name} decoded");
        for f in &m.fields {
            assert_eq!(m.get(&p, &f.name), None, "{name}.{} decoded", f.name);
        }
        let first = m.fields[0].name.as_str();
        let err = m.encode(&[(first, Value::Uint(1))]).unwrap_err();
        assert!(err.to_string().contains("out of scope"), "{err}");
    }
}

/// A newer dialect cannot slip a renamed or new target message past the
/// block: the name rule covers the whole MAVLink-M id range.
#[test]
fn name_rule_covers_newer_dialects() {
    let xml = r#"<mavlink><messages>
        <message id="53500" name="TARGET_PACKAGE_V2"><description>x</description>
          <field type="int32_t" name="lat" units="degE7">lat</field></message>
        <message id="53501" name="MUNITION_RELEASE"><description>x</description>
          <field type="uint8_t" name="store">s</field></message>
        <message id="53502" name="FRIENDLY_BEACON"><description>x</description>
          <field type="uint8_t" name="id">i</field></message>
        </messages></mavlink>"#;
    let d = mavlink_m::Dialect::parse("newer", xml).unwrap();
    for id in [53500, 53501] {
        let m = d.message(id).unwrap();
        assert!(m.is_blocked());
        assert!(m.decode(&[1, 2, 3, 4]).is_empty());
        let r = validate::check_payload(&d, id, &[1, 2, 3, 4][..m.max_len], true);
        assert!(r.framing_only, "{r}");
    }
    assert!(!d.message(53502).unwrap().is_blocked());
}

#[test]
fn every_cot_type_is_a_2525b_function() {
    let table = std::fs::read_to_string(data("2525b_function_ids.txt")).unwrap();
    let known: std::collections::HashSet<&str> =
        table.lines().filter(|l| !l.starts_with('#')).collect();
    for c in 0..=255u8 {
        if let Some(t) = mavlink_m::ppli_dimension(c) {
            let code = format!("{t} ------");
            assert!(
                known.contains(code.as_str()),
                "{t} is not a 2525B dimension"
            );
        }
        assert!("puafnsh".contains(mavlink_m::affiliation_from_identity(c)));
    }
}

#[test]
fn participant_position_is_a_friendly_marker() {
    let (id, p) = participant(1);
    let e = Gateway::default()
        .ingest(2, id, &p, 1_763_400_000_000_001)
        .unwrap();
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
    let (id, p) = participant(1);
    let f = frame(1, id, &p);
    let r = validate::check_frame(d, &f);
    assert!(r.is_ok() && r.findings.is_empty() && !r.framing_only, "{r}");
    assert_eq!(r.sys_id, Some(1));
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
    let (id, p) = participant(9);
    let r = validate::check_payload(d, id, &p, true);
    assert_eq!(
        errors(&r),
        ["ppli_type: 9 is not a defined MAVLINK_M_PPLI_TYPE value"]
    );

    // a protected (extended-Hamming) enum reports corruption and the nearest
    // codeword; the embedded dialect has none on an in-scope message, so
    // this uses a small dialect of its own
    let xml = r#"<mavlink><enums><enum name="BEACON_HEALTH">
          <entry value="0" name="BEACON_HEALTH_UNKNOWN"/>
          <entry value="15" name="BEACON_HEALTH_NOMINAL"/>
          <entry value="51" name="BEACON_HEALTH_DEGRADED"/>
          <entry value="60" name="BEACON_HEALTH_FAULT"/></enum></enums>
        <messages><message id="53502" name="FRIENDLY_BEACON"><description>x</description>
          <field type="uint8_t" name="health" enum="BEACON_HEALTH">h</field></message>
        </messages></mavlink>"#;
    let hd = mavlink_m::Dialect::parse("hamming", xml).unwrap();
    let r = validate::check_payload(&hd, 53502, &[14], true); // one bit from NOMINAL (15)
    let e = errors(&r).join("\n");
    assert!(
        e.contains("health: 14 is not a defined BEACON_HEALTH value: this enum's values are codewords at least 4 bits apart")
            && e.contains("nearest codeword BEACON_HEALTH_NOMINAL, 1 bits away"),
        "{e}"
    );

    // out-of-range latitude
    let (id, mut p) = participant(1);
    let lat = d.message(id).unwrap().field("lat").unwrap().offset;
    p[lat..lat + 4].copy_from_slice(&950_000_000i32.to_le_bytes());
    let r = validate::check_payload(d, id, &p, true);
    assert_eq!(
        errors(&r),
        ["lat: 950000000 degE7 is out of range (|value| <= 900000000)"]
    );

    // wrong checksum: a sender built from another definition
    let (id, p) = participant(1);
    let mut f = frame(1, id, &p);
    let n = f.len();
    f[n - 1] ^= 0xFF;
    let r = validate::check_frame(d, &f);
    assert!(errors(&r)[0].contains("does not match"), "{r}");

    // unknown id, too long, MAVLink 1
    assert!(!validate::check_payload(d, 53_999, &[1], true).is_ok());
    assert!(!validate::check_payload(d, id, &vec![1u8; 300], true).is_ok());
    assert!(!validate::check_payload(d, id, &p, false).is_ok());
}

/// The checker still checks framing, checksum and length of blocked
/// messages, but never decodes, range-checks or prints their fields.
#[test]
fn checker_does_not_decode_blocked_messages() {
    let d = mavlink_m::military();
    for name in MUST_BLOCK {
        let m = d.message_by_name(name).unwrap();
        // garbage fields: undefined enums, out-of-range positions
        let mut p = vec![0xEEu8; m.max_len];
        let r = validate::check_payload(d, m.id, &p, true);
        assert!(r.framing_only, "{r}");
        assert!(r.findings.iter().all(|f| f.field.is_none()), "{r}");
        assert!(r.is_ok(), "{r}");
        let text = r.to_string();
        for f in &m.fields {
            assert!(
                !text.contains(&format!("[{}]", f.name)),
                "{name}.{} printed: {text}",
                f.name
            );
        }
        // framing is still checked
        let mut fr = frame(1, m.id, &p);
        let n = fr.len();
        fr[n - 1] ^= 0xFF;
        assert!(!validate::check_frame(d, &fr).is_ok(), "{name} checksum");
        p.push(1);
        assert!(
            !validate::check_payload(d, m.id, &p, true).is_ok(),
            "{name} length"
        );
        assert!(!validate::check_payload(d, m.id, &p[..m.max_len.min(3)], false).is_ok());
    }
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
    assert!(d.message(0).is_some() && d.message(53003).is_some());
    let cmds = &d.enums["MAV_CMD"].entries;
    assert_eq!(cmds[0].name, "MAV_CMD_NAV_WAYPOINT");
    assert!(cmds.iter().any(|e| e.value == 53091));
    assert_eq!(mavlink_m::military().unresolved_includes, ["common.xml"]);
    std::fs::remove_dir_all(dir).ok();
}

/// Builds a .tlog with vehicle telemetry, a friendly position report, one
/// plausible frame of every blocked message and one corrupt position
/// report, then runs `uas2cot tlog` and `mavlinkm-check tlog` on it.
#[test]
fn uas2cot_and_checker_on_a_tlog() {
    let d = mavlink_m::military();
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
        let (pid, pp) = participant(1);
        w.write_frame(t0 + 100_000, &frame(1, pid, &pp)).unwrap();
        for (k, name) in MUST_BLOCK.iter().enumerate() {
            let m = d.message_by_name(name).unwrap();
            w.write_frame(t0 + 200_000 + k as u64, &frame(1, m.id, &plausible(m)))
                .unwrap();
        }
        // a corrupt one: undefined ppli_type
        let (pid, bad) = participant(9);
        w.write_frame(t0 + 300_000, &frame(1, pid, &bad)).unwrap();
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
        2,
        "only the vehicle and the friendly position report: {cot}"
    );
    assert!(
        lines[0].contains("uid=\"mav-1\" type=\"a-f-A-M-F-Q\""),
        "{}",
        lines[0]
    );
    assert!(
        lines[1].contains("uid=\"mavlink-m-ppli-2\" type=\"a-f-A\""),
        "{}",
        lines[1]
    );
    for word in ["target", "track-", "handover", "fire", "munition", "esad"] {
        assert!(
            !cot.to_lowercase().contains(word),
            "{word} in output: {cot}"
        );
    }

    let out = Command::new(env!("CARGO_BIN_EXE_mavlinkm-check"))
        .args(["tlog", p, "--all"])
        .output()
        .unwrap();
    assert_eq!(out.status.code(), Some(1));
    let report = String::from_utf8(out.stdout).unwrap();
    assert!(
        report.contains("PARTICIPANT_POSITION (53003) from 1/1\n  error [ppli_type]: 9 is not a defined MAVLINK_M_PPLI_TYPE value"),
        "{report}"
    );
    for name in MUST_BLOCK {
        let m = d.message_by_name(name).unwrap();
        assert!(
            report.contains(&format!(
                "{name} ({}) from 1/1 [out of scope: framing, length and checksum only]: ok",
                m.id
            )),
            "{name}: {report}"
        );
    }
    // no field of a blocked message is ever named in the report
    for line in report.lines().filter(|l| l.starts_with("  ")) {
        assert!(line.contains("[ppli_type]"), "unexpected finding: {line}");
    }
    let summary = String::from_utf8(out.stderr).unwrap();
    assert!(
        summary.contains("36 MAVLink-M messages checked against military"),
        "{summary}"
    );
    assert!(
        summary.contains("PARTICIPANT_POSITION: 2 checked, 1 with errors"),
        "{summary}"
    );
    std::fs::remove_file(path).ok();
}
