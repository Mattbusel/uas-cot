//! MAVLink to CoT without a MAVLink dependency: decode the two payloads
//! that matter for a map marker (`HEARTBEAT` for the vehicle type,
//! `GLOBAL_POSITION_INT` for position and velocity) from raw frame
//! payloads, as returned by `mavlink-tlog`, rust-mavlink or any parser.
//!
//! ```
//! use uas_cot::mavlink::{self, GlobalPositionInt, Options};
//!
//! let pos = GlobalPositionInt { time_boot_ms: 0, lat: 473_977_418, lon: 85_455_939, alt: 488_000,
//!     relative_alt: 0, vx: 500, vy: 0, vz: 0, hdg: 9000 };
//! let e = mavlink::vehicle(1, &pos, Some(2), 1_700_000_000_000_000, &Options::default()).unwrap();
//! assert_eq!(e.uid, "mav-1");
//! assert!((e.point.lat - 47.3977418).abs() < 1e-9);
//! assert_eq!(e.track.unwrap().course, 90.0);
//! ```

use crate::{Event, Point, Track};

/// `GLOBAL_POSITION_INT` message id.
pub const GLOBAL_POSITION_INT_ID: u32 = 33;
/// `HEARTBEAT` message id.
pub const HEARTBEAT_ID: u32 = 0;

/// `GLOBAL_POSITION_INT` (message 33), in wire units.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct GlobalPositionInt {
    /// Time since boot, ms.
    pub time_boot_ms: u32,
    /// Latitude, degrees * 1e7.
    pub lat: i32,
    /// Longitude, degrees * 1e7.
    pub lon: i32,
    /// Altitude above mean sea level, mm.
    pub alt: i32,
    /// Altitude above home, mm.
    pub relative_alt: i32,
    /// Ground X speed (north), cm/s.
    pub vx: i16,
    /// Ground Y speed (east), cm/s.
    pub vy: i16,
    /// Ground Z speed (down), cm/s.
    pub vz: i16,
    /// Heading, centidegrees; `u16::MAX` if unknown.
    pub hdg: u16,
}

impl GlobalPositionInt {
    /// Decodes a payload. MAVLink 2 trims trailing zero bytes, so shorter
    /// payloads are zero-padded; longer than 28 bytes is rejected.
    pub fn parse(payload: &[u8]) -> Option<Self> {
        if payload.len() > 28 {
            return None;
        }
        let mut b = [0u8; 28];
        b[..payload.len()].copy_from_slice(payload);
        let i32_at = |o: usize| i32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]);
        let i16_at = |o: usize| i16::from_le_bytes([b[o], b[o + 1]]);
        Some(GlobalPositionInt {
            time_boot_ms: u32::from_le_bytes([b[0], b[1], b[2], b[3]]),
            lat: i32_at(4),
            lon: i32_at(8),
            alt: i32_at(12),
            relative_alt: i32_at(16),
            vx: i16_at(20),
            vy: i16_at(22),
            vz: i16_at(24),
            hdg: u16::from_le_bytes([b[26], b[27]]),
        })
    }
}

/// `MAV_TYPE` from a `HEARTBEAT` payload (byte 4), if the payload is long enough.
pub fn heartbeat_mav_type(payload: &[u8]) -> Option<u8> {
    payload.get(4).copied()
}

/// How MAVLink vehicles become events.
#[derive(Clone, Debug, PartialEq)]
pub struct Options {
    /// Affiliation letter (default `'f'`: vehicles on your own link).
    pub affiliation: char,
    /// Seconds until stale (default 15).
    pub stale_secs: u64,
    /// UID prefix (default `"mav-"`); the system id is appended.
    pub uid_prefix: String,
}

impl Default for Options {
    fn default() -> Self {
        Options {
            affiliation: 'f',
            stale_secs: 15,
            uid_prefix: "mav-".into(),
        }
    }
}

/// CoT type for a `MAV_TYPE`: aircraft (fixed wing, multirotors,
/// helicopters, VTOLs, airships) are drones `a-?-A-M-F-Q`; ground rovers
/// `a-?-G-E-V`; boats `a-?-S`; submarines `a-?-U`; anything else `a-?-A`.
pub fn vehicle_type(mav_type: Option<u8>, affiliation: char) -> String {
    let tail = match mav_type {
        Some(
            1 | 2 | 3 | 4 | 7 | 8 | 13 | 14 | 15 | 16 | 19 | 20 | 21 | 22 | 23 | 24 | 25 | 28 | 29
            | 35,
        ) => "A-M-F-Q",
        Some(10) => "G-E-V",
        Some(11) => "S",
        Some(12) => "U",
        _ => "A",
    };
    format!("a-{affiliation}-{tail}")
}

/// A vehicle event from `GLOBAL_POSITION_INT` of system `sys_id`, or `None`
/// if the position is (0, 0) or out of range. Height is the MSL altitude.
pub fn vehicle(
    sys_id: u8,
    pos: &GlobalPositionInt,
    mav_type: Option<u8>,
    now_us: u64,
    opts: &Options,
) -> Option<Event> {
    if pos.lat == 0 && pos.lon == 0 {
        return None;
    }
    let point = Point::new(
        pos.lat as f64 * 1e-7,
        pos.lon as f64 * 1e-7,
        pos.alt as f64 / 1000.0,
    );
    if !point.is_valid() {
        return None;
    }
    let mut e = Event::new(
        format!("{}{sys_id}", opts.uid_prefix),
        vehicle_type(mav_type, opts.affiliation),
        now_us,
        opts.stale_secs,
    );
    e.point = point;
    e.callsign = Some(format!("MAV {sys_id}"));
    e.precision_location = Some(("GPS".into(), "MSL".into()));
    let (vn, ve) = (pos.vx as f64 / 100.0, pos.vy as f64 / 100.0);
    let speed = vn.hypot(ve);
    let course = if pos.hdg != u16::MAX {
        pos.hdg as f64 / 100.0
    } else {
        ve.atan2(vn).to_degrees().rem_euclid(360.0)
    };
    e.track = Some(Track { course, speed });
    e.remarks = Some(format!(
        "MAVLink system {sys_id}. Height above home: {:.1} m.",
        pos.relative_alt as f64 / 1000.0
    ));
    Some(e)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_trimmed_payload() {
        let full = GlobalPositionInt {
            time_boot_ms: 1,
            lat: 2,
            lon: 3,
            alt: 4,
            relative_alt: 5,
            vx: -6,
            vy: 7,
            vz: 8,
            hdg: 0,
        };
        let mut b = Vec::new();
        b.extend_from_slice(&1u32.to_le_bytes());
        for v in [2i32, 3, 4, 5] {
            b.extend_from_slice(&v.to_le_bytes());
        }
        for v in [-6i16, 7, 8] {
            b.extend_from_slice(&v.to_le_bytes());
        }
        // hdg = 0 trimmed by MAVLink 2
        assert_eq!(GlobalPositionInt::parse(&b), Some(full));
        assert_eq!(GlobalPositionInt::parse(&[0; 29]), None);
    }

    #[test]
    fn course_from_velocity_when_heading_unknown() {
        let p = GlobalPositionInt {
            lat: 1,
            lon: 1,
            vx: 0,
            vy: -300,
            hdg: u16::MAX,
            ..Default::default()
        };
        let e = vehicle(3, &p, None, 0, &Options::default()).unwrap();
        let t = e.track.unwrap();
        assert!((t.course - 270.0).abs() < 1e-9 && (t.speed - 3.0).abs() < 1e-9);
        assert_eq!(e.kind, "a-f-A");
    }
}
