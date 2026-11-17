//! Remote ID (ASTM F3411, ASD-STAN EN 4709-002) to CoT, using the
//! [`remoteid`] crate's merged per-aircraft record ([`remoteid::UasData`],
//! as kept by [`remoteid::Tracker`]).
//!
//! ```
//! use remoteid::{BasicId, IdType, Location, Message, UaType, UasData};
//! use uas_cot::rid::{self, Options};
//!
//! let mut uas = UasData::default();
//! let id = BasicId::new(IdType::SerialNumber, UaType::HelicopterOrMultirotor, "1596F35074529ABCDEFG").unwrap();
//! uas.update(&Message::BasicId(id));
//! let mut loc = Location::default();
//! loc.latitude = 45.5470818;
//! loc.longitude = -122.9668346;
//! loc.altitude_geo = Some(237.0);
//! uas.update(&Message::Location(loc));
//!
//! let e = rid::aircraft(&uas, None, 1_700_000_000_000_000, &Options::default()).unwrap();
//! assert_eq!(e.uid, "rid-1596F35074529ABCDEFG");
//! assert_eq!(e.kind, "a-u-A-M-F-Q");
//! assert_eq!(e.point.hae, 237.0);
//! ```

use crate::{Event, Point, Track, UNKNOWN_ERROR};
use remoteid::{UaType, UasData};
use std::fmt::Write as _;

/// How Remote ID records become events.
#[derive(Clone, Debug, PartialEq)]
pub struct Options {
    /// Affiliation letter in the CoT type: `'u'` unknown (default), `'n'`
    /// neutral, `'f'` friend, `'a'` assumed friend, `'s'` suspect, `'h'` hostile.
    pub affiliation: char,
    /// Seconds until the marker goes stale. Remote ID broadcasts at 1 Hz or
    /// faster, so the default is 30.
    pub stale_secs: u64,
    /// Prefix of the event UID (default `"rid-"`).
    pub uid_prefix: String,
}

impl Default for Options {
    fn default() -> Self {
        Options {
            affiliation: 'u',
            stale_secs: 30,
            uid_prefix: "rid-".into(),
        }
    }
}

/// The CoT type for an aircraft category: powered aircraft are drones
/// (`a-?-A-M-F-Q`, the MIL-STD-2525 "drone" function), lighter-than-air
/// craft are `a-?-A-C-L`, everything else is a generic air track `a-?-A`.
pub fn aircraft_type(ua: UaType, affiliation: char) -> String {
    let tail = match ua {
        UaType::FreeBalloon | UaType::CaptiveBalloon | UaType::Airship => "A-C-L",
        UaType::None
        | UaType::Aeroplane
        | UaType::HelicopterOrMultirotor
        | UaType::Gyroplane
        | UaType::HybridLift
        | UaType::Ornithopter
        | UaType::Glider
        | UaType::TetheredPoweredAircraft => "A-M-F-Q",
        _ => "A",
    };
    format!("a-{affiliation}-{tail}")
}

fn ua_type(data: &UasData) -> UaType {
    data.basic_id
        .iter()
        .flatten()
        .map(|b| b.ua_type)
        .find(|t| *t != UaType::None)
        .unwrap_or(UaType::None)
}

fn id_or<'a>(data: &'a UasData, fallback: Option<&'a str>) -> Option<&'a str> {
    data.primary_id().or(fallback).filter(|s| !s.is_empty())
}

/// The aircraft as an event, or `None` without a valid position or an id.
///
/// The UID is the prefix plus the first Basic ID (serial number,
/// registration ...), or `fallback_id` (for example the radio address)
/// when the aircraft has not sent a Basic ID yet. Height is the geodetic
/// altitude when sent, else the pressure altitude.
pub fn aircraft(
    data: &UasData,
    fallback_id: Option<&str>,
    now_us: u64,
    opts: &Options,
) -> Option<Event> {
    let id = id_or(data, fallback_id)?;
    let loc = data.location.as_ref()?;
    let (lat, lon) = loc.position()?;
    let mut point = Point::new(lat, lon, 0.0);
    if !point.is_valid() {
        return None;
    }
    let mut altsrc = "UNKNOWN";
    if let Some(h) = loc.altitude_geo {
        point.hae = h as f64;
        altsrc = "GPS";
    } else if let Some(h) = loc.altitude_baro {
        point.hae = h as f64;
        altsrc = "BARO";
    }
    point.ce = loc
        .horizontal_accuracy
        .meters()
        .map_or(UNKNOWN_ERROR, f64::from);
    point.le = loc
        .vertical_accuracy
        .meters()
        .map_or(UNKNOWN_ERROR, f64::from);

    let mut e = Event::new(
        format!("{}{id}", opts.uid_prefix),
        aircraft_type(ua_type(data), opts.affiliation),
        now_us,
        opts.stale_secs,
    );
    e.point = point;
    e.callsign = Some(id.to_string());
    e.precision_location = Some(("GPS".into(), altsrc.into()));
    if let (Some(course), Some(speed)) = (loc.direction, loc.speed_horizontal) {
        e.track = Some(Track {
            course: course as f64,
            speed: speed as f64,
        });
    }
    let mut r = format!(
        "Remote ID. UA type: {:?}. Status: {:?}.",
        ua_type(data),
        loc.status
    );
    if let Some(h) = loc.height {
        let _ = write!(r, " Height: {h:.1} m ({:?}).", loc.height_type);
    }
    if let Some(v) = loc.speed_vertical {
        let _ = write!(r, " Vertical speed: {v:.1} m/s.");
    }
    if let Some(op) = data
        .operator_id
        .as_ref()
        .and_then(|o| o.operator_id_str())
        .filter(|s| !s.is_empty())
    {
        let _ = write!(r, " Operator ID: {op}.");
    }
    if let Some(d) = data
        .self_id
        .as_ref()
        .and_then(|s| s.description_str())
        .filter(|s| !s.is_empty())
    {
        let _ = write!(r, " Self ID: {d}.");
    }
    e.remarks = Some(r);
    Some(e)
}

/// The operator (or takeoff point) from the System message, as a spot
/// marker `b-m-p-s-m`, or `None` when the System message has no position.
pub fn operator(
    data: &UasData,
    fallback_id: Option<&str>,
    now_us: u64,
    opts: &Options,
) -> Option<Event> {
    let id = id_or(data, fallback_id)?;
    let sys = data.system.as_ref()?;
    let (lat, lon) = sys.operator_position()?;
    let mut point = Point::new(lat, lon, sys.operator_altitude_geo.map_or(0.0, f64::from));
    if !point.is_valid() {
        return None;
    }
    point.ce = UNKNOWN_ERROR;
    let mut e = Event::new(
        format!("{}op-{id}", opts.uid_prefix),
        "b-m-p-s-m",
        now_us,
        opts.stale_secs,
    );
    e.point = point;
    let op = data
        .operator_id
        .as_ref()
        .and_then(|o| o.operator_id_str())
        .filter(|s| !s.is_empty());
    e.callsign = Some(match op {
        Some(op) => format!("{op} (operator of {id})"),
        None => format!("Operator of {id}"),
    });
    e.remarks = Some(format!(
        "Remote ID operator location ({:?}).",
        sys.operator_location_type
    ));
    Some(e)
}

/// Both events for one aircraft (aircraft first), skipping the ones that
/// cannot be built yet.
pub fn events(
    data: &UasData,
    fallback_id: Option<&str>,
    now_us: u64,
    opts: &Options,
) -> Vec<Event> {
    aircraft(data, fallback_id, now_us, opts)
        .into_iter()
        .chain(operator(data, fallback_id, now_us, opts))
        .collect()
}
