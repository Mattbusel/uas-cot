//! Turn drone telemetry into Cursor on Target (CoT) events for TAK (ATAK,
//! WinTAK, iTAK, TAK Server, FreeTAKServer, OpenTAKServer).
//!
//! | Source | Module | Events |
//! |---|---|---|
//! | Drone Remote ID (ASTM F3411 / EN 4709-002) via the [`remoteid`](https://docs.rs/remoteid) crate | [`rid`] | the aircraft, and its operator |
//! | MISB ST 0601 KLV from drone video via the [`misb`](https://docs.rs/misb) crate | [`fmv`] | the platform with its sensor field of view, and the sensor point of interest |
//! | MAVLink `GLOBAL_POSITION_INT` / `HEARTBEAT` payloads | [`mavlink`] | the vehicle |
//! | MAVLink-M (`military.xml`) `PARTICIPANT_POSITION`, feature `mavlink-m` | `mavlink_m` | friendly position reports, plus a framing and checksum checker; target, track, fires and engagement messages are refused |
//!
//! [`Event`] is the CoT 2.0 event model; [`Event::to_xml`] writes the XML
//! that TAK clients and servers accept (validated against the MITRE public
//! release schemas in the tests). [`net`] sends events to the SA multicast
//! group or a TAK Server TCP input.
//!
//! ```
//! use uas_cot::{Event, Point, Track};
//!
//! let mut e = Event::new("rid-1596F35074529ABCDEFG", "a-u-A-M-F-Q", 1_700_000_000_000_000, 30);
//! e.point = Point::new(38.8895, -77.0353, 120.0);
//! e.callsign = Some("1596F35074529ABCDEFG".into());
//! e.track = Some(Track { course: 92.0, speed: 11.5 });
//! let xml = e.to_xml();
//! assert!(xml.starts_with("<?xml version=\"1.0\" encoding=\"UTF-8\"?><event version=\"2.0\""));
//! assert!(xml.contains("<point lat=\"38.8895000\" lon=\"-77.0353000\" hae=\"120.0\""));
//! ```
#![forbid(unsafe_code)]
#![warn(missing_docs)]

use std::fmt::{self, Write as _};

#[cfg(feature = "misb")]
pub mod fmv;
pub mod mavlink;
#[cfg(feature = "mavlink-m")]
pub mod mavlink_m;
pub mod net;
#[cfg(feature = "remoteid")]
pub mod rid;

/// The [`remoteid`](https://docs.rs/remoteid) crate this build of uas-cot
/// uses. Name its types through this path (`uas_cot::remoteid::Tracker`)
/// and they always match what [`rid`] accepts, with no second copy of the
/// crate in your dependency tree.
#[cfg(feature = "remoteid")]
pub use remoteid;

/// The [`misb`](https://docs.rs/misb) crate this build of uas-cot uses, for
/// the packets [`fmv`] accepts (`uas_cot::misb::st0601::Packet`).
#[cfg(feature = "misb")]
pub use misb;

#[cfg(all(doctest, feature = "remoteid"))]
#[doc = include_str!("../docs/integration.md")]
struct IntegrationDoctests;

/// "Unknown" circular and linear error, as CoT defines it.
pub const UNKNOWN_ERROR: f64 = 9_999_999.0;

/// A WGS84 point. `hae` is metres above the ellipsoid; `ce` and `le` are
/// the 1-sigma horizontal and vertical errors in metres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point {
    /// Latitude, degrees.
    pub lat: f64,
    /// Longitude, degrees.
    pub lon: f64,
    /// Height above the WGS84 ellipsoid, metres.
    pub hae: f64,
    /// Circular error, metres.
    pub ce: f64,
    /// Linear (vertical) error, metres.
    pub le: f64,
}

impl Point {
    /// A point with unknown errors.
    pub fn new(lat: f64, lon: f64, hae: f64) -> Self {
        Point {
            lat,
            lon,
            hae,
            ce: UNKNOWN_ERROR,
            le: UNKNOWN_ERROR,
        }
    }

    /// True if latitude and longitude are finite and in range.
    pub fn is_valid(&self) -> bool {
        self.lat.is_finite()
            && self.lon.is_finite()
            && self.lat.abs() <= 90.0
            && self.lon.abs() <= 180.0
    }
}

impl Default for Point {
    fn default() -> Self {
        Point::new(0.0, 0.0, 0.0)
    }
}

/// `<track>`: course over ground (degrees true) and speed (m/s).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Track {
    /// Course, degrees clockwise from true north.
    pub course: f64,
    /// Speed, metres per second.
    pub speed: f64,
}

/// `<sensor>`: where a camera on the platform is looking. ATAK draws it as
/// a field of view wedge.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Sensor {
    /// Azimuth of the line of sight, degrees true.
    pub azimuth: Option<f64>,
    /// Elevation of the line of sight, degrees (-90 to 90).
    pub elevation: Option<f64>,
    /// Roll, degrees (-180 to 180).
    pub roll: Option<f64>,
    /// Horizontal field of view, degrees.
    pub fov: Option<f64>,
    /// Vertical field of view, degrees.
    pub vfov: Option<f64>,
    /// Range to the scene, metres.
    pub range: Option<f64>,
    /// Bearing of image north, degrees true (ATAK `north`).
    pub north: Option<f64>,
    /// Sensor name.
    pub model: Option<String>,
    /// Sensor type (`"r-e"` electro-optical, `"r-i"` infrared ...).
    pub kind: Option<String>,
}

/// `<link>`: a reference to another event (for example the platform that
/// produced a sensor point of interest).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Link {
    /// UID of the other event.
    pub uid: String,
    /// CoT type of the other event.
    pub kind: String,
    /// Relation, for example `"p-p"` (parent-producer).
    pub relation: String,
}

/// A CoT 2.0 event.
#[derive(Clone, Debug, PartialEq)]
pub struct Event {
    /// Unique, stable id of the thing described.
    pub uid: String,
    /// CoT type, for example `"a-u-A-M-F-Q"`.
    pub kind: String,
    /// How the position was obtained (`"m-g"` = GPS).
    pub how: String,
    /// Event time, microseconds since 1970 UTC.
    pub time_us: u64,
    /// Start of validity, microseconds since 1970 UTC.
    pub start_us: u64,
    /// End of validity, microseconds since 1970 UTC.
    pub stale_us: u64,
    /// Position.
    pub point: Point,
    /// `<contact callsign>`.
    pub callsign: Option<String>,
    /// `<track>`.
    pub track: Option<Track>,
    /// `<sensor>`.
    pub sensor: Option<Sensor>,
    /// `<link>` elements.
    pub links: Vec<Link>,
    /// `<remarks>` text.
    pub remarks: Option<String>,
    /// `<__video url>`: a video stream TAK clients can open from the marker.
    pub video_url: Option<String>,
    /// `<precisionlocation geopointsrc altsrc>`, for example `("GPS", "GPS")`.
    pub precision_location: Option<(String, String)>,
}

impl Event {
    /// A new event of `kind` at `time_us`, stale after `stale_secs`
    /// seconds, with `how = "m-g"`.
    pub fn new(
        uid: impl Into<String>,
        kind: impl Into<String>,
        time_us: u64,
        stale_secs: u64,
    ) -> Self {
        Event {
            uid: uid.into(),
            kind: kind.into(),
            how: "m-g".into(),
            time_us,
            start_us: time_us,
            stale_us: time_us.saturating_add(stale_secs.saturating_mul(1_000_000)),
            point: Point::default(),
            callsign: None,
            track: None,
            sensor: None,
            links: Vec::new(),
            remarks: None,
            video_url: None,
            precision_location: None,
        }
    }

    /// The event as a CoT XML document (one line, with XML declaration).
    pub fn to_xml(&self) -> String {
        let mut s = String::with_capacity(512);
        self.write_xml(&mut s)
            .expect("writing to a String does not fail");
        s
    }

    /// Writes the event as a CoT XML document.
    pub fn write_xml(&self, w: &mut impl fmt::Write) -> fmt::Result {
        w.write_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>")?;
        write!(
            w,
            "<event version=\"2.0\" uid=\"{}\" type=\"{}\" how=\"{}\" time=\"{}\" start=\"{}\" stale=\"{}\">",
            Esc(&self.uid),
            Esc(&self.kind),
            Esc(&self.how),
            Iso8601(self.time_us),
            Iso8601(self.start_us),
            Iso8601(self.stale_us)
        )?;
        let p = &self.point;
        write!(
            w,
            "<point lat=\"{:.7}\" lon=\"{:.7}\" hae=\"{:.1}\" ce=\"{:.1}\" le=\"{:.1}\"/>",
            p.lat,
            p.lon,
            finite_or(p.hae, 0.0),
            finite_or(p.ce, UNKNOWN_ERROR).max(0.0),
            finite_or(p.le, UNKNOWN_ERROR).max(0.0)
        )?;
        w.write_str("<detail>")?;
        if let Some(c) = &self.callsign {
            write!(w, "<contact callsign=\"{}\"/>", Esc(c))?;
        }
        if let Some((g, a)) = &self.precision_location {
            write!(
                w,
                "<precisionlocation geopointsrc=\"{}\" altsrc=\"{}\"/>",
                Esc(g),
                Esc(a)
            )?;
        }
        if let Some(t) = &self.track {
            if t.course.is_finite() && t.speed.is_finite() {
                write!(
                    w,
                    "<track course=\"{:.1}\" speed=\"{:.2}\"/>",
                    wrap360(t.course, 1),
                    t.speed.max(0.0)
                )?;
            }
        }
        if let Some(sn) = &self.sensor {
            w.write_str("<sensor")?;
            if let Some(v) = sn.azimuth.filter(|v| v.is_finite()) {
                write!(w, " azimuth=\"{:.2}\"", wrap360(v, 2))?;
            }
            if let Some(v) = sn.elevation.filter(|v| v.is_finite() && v.abs() <= 90.0) {
                write!(w, " elevation=\"{v:.2}\"")?;
            }
            if let Some(v) = sn.roll.filter(|v| v.is_finite()) {
                let r = wrap360(v, 2);
                write!(w, " roll=\"{:.2}\"", if r > 180.0 { r - 360.0 } else { r })?;
            }
            if let Some(v) = sn.fov.filter(|v| v.is_finite() && *v >= 0.0 && *v < 360.0) {
                write!(w, " fov=\"{v:.3}\"")?;
            }
            if let Some(v) = sn.vfov.filter(|v| v.is_finite() && *v >= 0.0 && *v < 360.0) {
                write!(w, " vfov=\"{v:.3}\"")?;
            }
            if let Some(v) = sn.range.filter(|v| v.is_finite() && *v >= 0.0) {
                write!(w, " range=\"{v:.1}\"")?;
            }
            if let Some(v) = sn.north.filter(|v| v.is_finite()) {
                write!(w, " north=\"{:.2}\"", wrap360(v, 2))?;
            }
            if let Some(v) = &sn.model {
                write!(w, " model=\"{}\"", Esc(v))?;
            }
            if let Some(v) = &sn.kind {
                write!(w, " type=\"{}\"", Esc(v))?;
            }
            w.write_str("/>")?;
        }
        for l in &self.links {
            write!(
                w,
                "<link uid=\"{}\" type=\"{}\" relation=\"{}\"/>",
                Esc(&l.uid),
                Esc(&l.kind),
                Esc(&l.relation)
            )?;
        }
        if let Some(u) = &self.video_url {
            write!(w, "<__video url=\"{}\"/>", Esc(u))?;
        }
        if let Some(r) = &self.remarks {
            write!(w, "<remarks>{}</remarks>", Esc(r))?;
        }
        w.write_str("</detail></event>")
    }
}

fn finite_or(v: f64, d: f64) -> f64 {
    if v.is_finite() {
        v
    } else {
        d
    }
}

/// Rounds to `decimals` places, then wraps into `0..360`.
fn wrap360(v: f64, decimals: i32) -> f64 {
    let p = 10f64.powi(decimals);
    let r = ((v * p).round() / p).rem_euclid(360.0) + 0.0; // + 0.0 turns -0.0 into 0.0
    if r >= 360.0 {
        0.0
    } else {
        r
    }
}

/// XML attribute and text escaping. Control characters XML 1.0 cannot carry
/// are dropped.
struct Esc<'a>(&'a str);

impl fmt::Display for Esc<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for c in self.0.chars() {
            match c {
                '&' => f.write_str("&amp;")?,
                '<' => f.write_str("&lt;")?,
                '>' => f.write_str("&gt;")?,
                '"' => f.write_str("&quot;")?,
                '\'' => f.write_str("&apos;")?,
                '\t' | '\n' | '\r' => write!(f, "&#{};", c as u32)?,
                c if (c as u32) < 0x20 || c == '\u{FFFE}' || c == '\u{FFFF}' => {}
                c => f.write_char(c)?,
            }
        }
        Ok(())
    }
}

/// Formats microseconds since 1970 as `YYYY-MM-DDTHH:MM:SS.mmmZ`.
pub struct Iso8601(pub u64);

impl fmt::Display for Iso8601 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let secs = self.0 / 1_000_000;
        let ms = (self.0 % 1_000_000) / 1000;
        let days = (secs / 86_400) as i64;
        let rem = secs % 86_400;
        // civil_from_days, H. Hinnant
        let z = days + 719_468;
        let era = z.div_euclid(146_097);
        let doe = z - era * 146_097;
        let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
        let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
        let mp = (5 * doy + 2) / 153;
        let d = doy - (153 * mp + 2) / 5 + 1;
        let m = if mp < 10 { mp + 3 } else { mp - 9 };
        let y = yoe + era * 400 + i64::from(m <= 2);
        write!(
            f,
            "{y:04}-{m:02}-{d:02}T{:02}:{:02}:{:02}.{ms:03}Z",
            rem / 3600,
            rem % 3600 / 60,
            rem % 60
        )
    }
}

/// Microseconds since 1970 UTC, from the system clock.
pub fn now_us() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn iso8601() {
        assert_eq!(Iso8601(0).to_string(), "1970-01-01T00:00:00.000Z");
        assert_eq!(
            Iso8601(1_231_798_102_000_000).to_string(),
            "2009-01-12T22:08:22.000Z"
        );
        assert_eq!(
            Iso8601(951_782_400_123_456).to_string(),
            "2000-02-29T00:00:00.123Z"
        );
        assert_eq!(
            Iso8601(4_102_444_799_999_999).to_string(),
            "2099-12-31T23:59:59.999Z"
        );
    }

    #[test]
    fn escaping() {
        let mut e = Event::new("a\"<b>&'c", "a-u-A", 0, 1);
        e.remarks = Some("line1\nline2\u{1}".into());
        let x = e.to_xml();
        assert!(x.contains("uid=\"a&quot;&lt;b&gt;&amp;&apos;c\""));
        assert!(x.contains("<remarks>line1&#10;line2</remarks>"));
    }

    #[test]
    fn angles_are_normalised() {
        let mut e = Event::new("x", "a-u-A", 0, 1);
        e.track = Some(Track {
            course: -0.0000001,
            speed: -3.0,
        });
        e.sensor = Some(Sensor {
            azimuth: Some(370.0),
            roll: Some(270.0),
            elevation: Some(-120.0),
            ..Default::default()
        });
        let x = e.to_xml();
        assert!(x.contains("<track course=\"0.0\" speed=\"0.00\"/>"), "{x}");
        assert!(x.contains("azimuth=\"10.00\" roll=\"-90.00\"/>"), "{x}");
        e.sensor = Some(Sensor {
            north: Some(-90.0),
            ..Default::default()
        });
        assert!(e.to_xml().contains("<sensor north=\"270.00\"/>"));
    }
}
