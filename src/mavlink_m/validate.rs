//! Conformance checks of MAVLink frames and payloads against a dialect
//! definition (by default the embedded MAVLink-M `military.xml`).
//!
//! Every check comes from the XML itself, so a newer `military.xml` loaded
//! with [`Dialect::from_file`] is checked by its own rules:
//!
//! | Check | Source in the XML |
//! |---|---|
//! | message id is defined | `<message id>` |
//! | payload length (MAVLink 1: exact; MAVLink 2: not longer than the message, trailing zeros trimmed) | field types and `<extensions/>` |
//! | checksum, with the message's `CRC_EXTRA` | field names, types and order |
//! | enum fields hold a defined value | `enum=` and `<entry value>` |
//! | protected enums: an undefined value is reported as corruption, with the distance to the nearest valid codeword | entries whose pairwise Hamming distance is 4 or more |
//! | bitmask fields set no undefined bits | `bitmask="true"` |
//! | `degE7` latitude and longitude are in range | `units="degE7"` |
//! | ranges stated in descriptions: `[0.0-1.0]`, `[0-100]` (with `255 = N/A`), `0 to 10000` | field `<description>` |
//! | fields described "Never 0" are not 0 | field `<description>` |
//! | `char[N]` text is valid UTF-8 | field types |
//! | a track UID is not all zero, unless the description allows it | field `<description>` |
//!
//! Out-of-scope messages (target, track, handover, fires, munition, arming,
//! weapon, engagement, see [`super::scope`]) get the frame checks only: id,
//! length and checksum. Their fields are never decoded, range-checked or
//! printed; the report marks them [`Report::framing_only`].

use super::dialect::{crc_accumulate, Dialect, Field, FieldType, Message, Value};
use std::fmt;

/// How serious a finding is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    /// Allowed by the definition but worth a look.
    Warning,
    /// The message does not conform to the definition.
    Error,
}

/// One problem found in a message.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Finding {
    /// Severity.
    pub severity: Severity,
    /// Field the finding is about, if any.
    pub field: Option<String>,
    /// What is wrong.
    pub message: String,
}

/// The result of checking one message.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Report {
    /// Message id.
    pub msg_id: u32,
    /// Message name, if the id is defined.
    pub name: Option<String>,
    /// Sender system id (frames only).
    pub sys_id: Option<u8>,
    /// Sender component id (frames only).
    pub comp_id: Option<u8>,
    /// Findings, in check order.
    pub findings: Vec<Finding>,
    /// True for an out-of-scope message ([`super::scope::is_blocked`]):
    /// only id, length and checksum were checked; no field was decoded.
    pub framing_only: bool,
}

impl Report {
    /// True if there is no [`Severity::Error`] finding.
    pub fn is_ok(&self) -> bool {
        self.findings.iter().all(|f| f.severity != Severity::Error)
    }

    fn push(&mut self, severity: Severity, field: Option<&str>, message: String) {
        self.findings.push(Finding {
            severity,
            field: field.map(str::to_string),
            message,
        });
    }
}

impl fmt::Display for Report {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let name = self.name.as_deref().unwrap_or("?");
        write!(f, "{} ({})", name, self.msg_id)?;
        if let (Some(s), Some(c)) = (self.sys_id, self.comp_id) {
            write!(f, " from {s}/{c}")?;
        }
        if self.framing_only {
            write!(f, " [out of scope: framing, length and checksum only]")?;
        }
        if self.findings.is_empty() {
            return write!(f, ": ok");
        }
        for x in &self.findings {
            let sev = match x.severity {
                Severity::Error => "error",
                Severity::Warning => "warning",
            };
            write!(f, "\n  {sev}")?;
            if let Some(fl) = &x.field {
                write!(f, " [{fl}]")?;
            }
            write!(f, ": {}", x.message)?;
        }
        Ok(())
    }
}

const MAGIC_V1: u8 = 0xFE;
const MAGIC_V2: u8 = 0xFD;

/// Length of the frame at the start of `data`, if it is complete.
pub fn frame_len(data: &[u8]) -> Option<usize> {
    match *data.first()? {
        MAGIC_V1 => Some(8 + *data.get(1)? as usize),
        MAGIC_V2 => {
            let len = *data.get(1)? as usize;
            let signed = *data.get(2)? & 0x01 != 0;
            Some(12 + len + if signed { 13 } else { 0 })
        }
        _ => None,
    }
    .filter(|&n| n <= data.len())
}

/// Checks one complete MAVLink 1 or 2 frame (start byte to checksum, plus
/// the signature if the frame is signed). The signature itself is not
/// verified; that needs the link's secret key.
pub fn check_frame(d: &Dialect, frame: &[u8]) -> Report {
    let mut r = Report {
        msg_id: 0,
        name: None,
        sys_id: None,
        comp_id: None,
        findings: Vec::new(),
        framing_only: false,
    };
    let Some(&magic) = frame.first() else {
        r.push(Severity::Error, None, "empty frame".into());
        return r;
    };
    let (hdr, v2) = match magic {
        MAGIC_V1 => (6, false),
        MAGIC_V2 => (10, true),
        m => {
            r.push(Severity::Error, None, format!("bad start byte 0x{m:02X}"));
            return r;
        }
    };
    let Some(total) = frame_len(frame) else {
        r.push(Severity::Error, None, "truncated frame".into());
        return r;
    };
    let len = frame[1] as usize;
    if v2 {
        if frame[2] & !0x01 != 0 {
            r.push(
                Severity::Error,
                None,
                format!("unknown incompatibility flags 0x{:02X}", frame[2]),
            );
        }
        r.sys_id = Some(frame[5]);
        r.comp_id = Some(frame[6]);
        r.msg_id = u32::from_le_bytes([frame[7], frame[8], frame[9], 0]);
    } else {
        r.sys_id = Some(frame[3]);
        r.comp_id = Some(frame[4]);
        r.msg_id = frame[5] as u32;
    }
    let payload = &frame[hdr..hdr + len];
    let found = u16::from_le_bytes([frame[hdr + len], frame[hdr + len + 1]]);
    let inner = check_payload(d, r.msg_id, payload, v2);
    r.name = inner.name.clone();
    r.framing_only = inner.framing_only;
    if let Some(m) = d.message(r.msg_id) {
        let mut crc = 0xFFFF;
        for &b in &frame[1..hdr + len] {
            crc = crc_accumulate(b, crc);
        }
        crc = crc_accumulate(m.crc_extra, crc);
        if crc != found {
            r.push(
                Severity::Error,
                None,
                format!(
                    "checksum 0x{found:04X} does not match 0x{crc:04X} (CRC_EXTRA {} from the definition): the sender uses a different definition of {} or the frame is corrupt",
                    m.crc_extra, m.name
                ),
            );
        }
    }
    if total != frame.len() {
        r.push(
            Severity::Warning,
            None,
            format!("{} bytes after the frame", frame.len() - total),
        );
    }
    r.findings.extend(inner.findings);
    r
}

/// Checks a payload of message `msg_id`. `v2` selects the MAVLink 2 length
/// rules (trailing zero bytes trimmed).
pub fn check_payload(d: &Dialect, msg_id: u32, payload: &[u8], v2: bool) -> Report {
    let mut r = Report {
        msg_id,
        name: None,
        sys_id: None,
        comp_id: None,
        findings: Vec::new(),
        framing_only: false,
    };
    let Some(m) = d.message(msg_id) else {
        r.push(
            Severity::Error,
            None,
            format!("message id {msg_id} is not defined in {}", d.name),
        );
        return r;
    };
    r.name = Some(m.name.clone());
    check_length(m, payload, v2, &mut r);
    if m.is_blocked() {
        // out of scope: no field is decoded, checked or reported
        r.framing_only = true;
        return r;
    }
    for f in &m.fields {
        let v = super::dialect::decode_field(f, payload);
        check_field(d, f, &v, &mut r);
    }
    r
}

fn check_length(m: &Message, payload: &[u8], v2: bool, r: &mut Report) {
    let n = payload.len();
    if v2 {
        if n > m.max_len {
            r.push(
                Severity::Error,
                None,
                format!("payload is {n} bytes; {} is at most {}", m.name, m.max_len),
            );
        } else if n == 0 {
            r.push(
                Severity::Error,
                None,
                "MAVLink 2 payloads carry at least one byte".into(),
            );
        } else if n > 1 && payload[n - 1] == 0 {
            r.push(
                Severity::Warning,
                None,
                "trailing zero bytes not trimmed (MAVLink 2 senders trim them)".into(),
            );
        }
    } else {
        if m.id > 255 {
            r.push(
                Severity::Error,
                None,
                format!("message id {} does not fit a MAVLink 1 frame", m.id),
            );
        }
        if n != m.base_len {
            r.push(
                Severity::Error,
                None,
                format!(
                    "MAVLink 1 payload is {n} bytes; {} is {}",
                    m.name, m.base_len
                ),
            );
        }
    }
}

fn check_field(d: &Dialect, f: &Field, v: &Value, r: &mut Report) {
    let name = Some(f.name.as_str());
    let desc = f.description.as_str();
    // enums
    if let Some(en) = f.enum_name.as_deref() {
        let values: Vec<u64> = match v {
            Value::Array(items) => items.iter().filter_map(Value::as_u64).collect(),
            other => other.as_u64().into_iter().collect(),
        };
        match d.enums.get(en) {
            None => r.push(
                Severity::Warning,
                name,
                format!("enum {en} is not defined in {} (missing include?)", d.name),
            ),
            Some(e) if e.bitmask => {
                let all = e.all_bits();
                for x in values {
                    if x & !all != 0 {
                        r.push(
                            Severity::Error,
                            name,
                            format!("bits 0x{:X} are not defined in bitmask {en}", x & !all),
                        );
                    }
                }
            }
            Some(e) => {
                for x in values {
                    if e.entry(x).is_some() {
                        continue;
                    }
                    let protected = e.entries.len() >= 3 && e.min_hamming_distance() >= Some(4);
                    let mut msg = format!("{x} is not a defined {en} value");
                    if protected {
                        let near = e
                            .entries
                            .iter()
                            .map(|y| ((y.value ^ x).count_ones(), &y.name))
                            .min();
                        msg.push_str(
                            ": this enum's values are codewords at least 4 bits apart, so the field is corrupt; reject it, do not correct it",
                        );
                        if let Some((dist, n)) = near {
                            msg.push_str(&format!(" (nearest codeword {n}, {dist} bits away)"));
                        }
                    }
                    r.push(Severity::Error, name, msg);
                }
            }
        }
    }
    // latitude / longitude
    if f.units.as_deref() == Some("degE7") {
        if let Some(x) = v.as_i64() {
            let limit = if f.name.contains("lon") {
                1_800_000_000
            } else if f.name.contains("lat") {
                900_000_000
            } else {
                i64::MAX
            };
            if x.abs() > limit {
                r.push(
                    Severity::Error,
                    name,
                    format!("{x} degE7 is out of range (|value| <= {limit})"),
                );
            }
        }
    }
    // description-stated ranges
    if let Some(x) = v.as_f64() {
        if desc.contains("[0.0-1.0]") {
            if x.is_nan() {
                if !desc.contains("NaN") {
                    r.push(
                        Severity::Warning,
                        name,
                        "NaN where the description gives the range [0.0-1.0]".into(),
                    );
                }
            } else if !(0.0..=1.0).contains(&x) {
                r.push(Severity::Error, name, format!("{x} is outside [0.0-1.0]"));
            }
        }
        if desc.contains("[0-100]") {
            let na = desc.contains("255 = N/A") && x == 255.0;
            if !na && !(0.0..=100.0).contains(&x) {
                r.push(Severity::Error, name, format!("{x} is outside [0-100]"));
            }
        }
        if desc.contains("0 to 10000") && x > 10_000.0 {
            r.push(Severity::Error, name, format!("{x} is above 10000"));
        }
        if desc.contains("Never 0") && x == 0.0 {
            r.push(
                Severity::Error,
                name,
                "is 0; the definition says it is never 0".into(),
            );
        }
        if f.ty == FieldType::F32 && x.is_infinite() {
            r.push(Severity::Error, name, "is infinite".into());
        }
    }
    // text
    if let (FieldType::Char, Some(_), Value::Text(s)) = (f.ty, f.array_len, v) {
        if s.contains('\u{FFFD}') {
            r.push(Severity::Warning, name, "text is not valid UTF-8".into());
        }
    }
    // track UIDs
    if f.name.ends_with("track_uid") && !f.name.starts_with("parent") {
        if let Some(b) = v.as_bytes() {
            if b.iter().all(|&x| x == 0) && !desc.contains("All-zero if") {
                r.push(
                    Severity::Warning,
                    name,
                    "track UID is all zero, so it cannot correlate this track".into(),
                );
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_len_handles_signed_and_short() {
        assert_eq!(frame_len(&[]), None);
        assert_eq!(
            frame_len(&[0xFD, 1, 0, 0, 0, 0, 0, 0, 0, 0, 9, 0, 0]),
            Some(13)
        );
        assert_eq!(frame_len(&[0xFD, 1, 1, 0, 0, 0, 0, 0, 0, 0, 9, 0, 0]), None);
        assert_eq!(frame_len(&[0x00, 1]), None);
    }
}
