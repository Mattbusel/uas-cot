//! MISB ST 0601 (the KLV metadata in STANAG 4609 drone video) to CoT, using
//! the [`misb`] crate.
//!
//! [`platform`] is the aircraft with a `<sensor>` element (ATAK draws the
//! camera field of view from it) and, optionally, the video URL.
//! [`spi`] is the sensor point of interest: where the center of the picture
//! is on the ground, linked to the platform.
//!
//! ```
//! use misb::st0601::{Builder, Packet, Tag};
//! use uas_cot::fmv::{self, Options};
//!
//! let mut b = Builder::new(1_700_000_000_000_000);
//! b.text(Tag::PlatformTailNumber, "N101").unwrap()
//!     .float(Tag::SensorLatitude, 38.8895).unwrap()
//!     .float(Tag::SensorLongitude, -77.0353).unwrap()
//!     .float(Tag::SensorTrueAltitude, 450.0).unwrap()
//!     .float(Tag::FrameCenterLatitude, 38.8950).unwrap()
//!     .float(Tag::FrameCenterLongitude, -77.0300).unwrap();
//! let bytes = b.finish();
//! let (packet, _) = Packet::parse(&bytes).unwrap();
//!
//! let opts = Options::default();
//! let air = fmv::platform(&packet, &opts).unwrap();
//! let spi = fmv::spi(&packet, &opts).unwrap();
//! assert_eq!(air.uid, "fmv-N101");
//! assert_eq!(spi.uid, "fmv-N101.SPI1");
//! assert_eq!(spi.links[0].uid, "fmv-N101");
//! ```

use crate::{Event, Link, Point, Sensor, Track};
use misb::st0601::{Packet, Tag};
use std::fmt::Write as _;

/// How KLV packets become events.
#[derive(Clone, Debug, PartialEq)]
pub struct Options {
    /// Affiliation letter for the platform type (default `'f'`: the video
    /// usually comes from your own aircraft).
    pub affiliation: char,
    /// Seconds until the markers go stale (default 10).
    pub stale_secs: u64,
    /// Platform UID. Default: `"fmv-"` plus the tail number, call sign,
    /// designation or mission ID from the packet, in that order, or
    /// `"fmv-UAS"` when the packet has none of them (many encoders send
    /// none: give each stream its own UID then).
    pub uid: Option<String>,
    /// Video stream URL to attach to the platform (`<__video url>`).
    pub video_url: Option<String>,
}

impl Default for Options {
    fn default() -> Self {
        Options {
            affiliation: 'f',
            stale_secs: 10,
            uid: None,
            video_url: None,
        }
    }
}

fn name<'a>(p: &Packet<'a>) -> Option<&'a str> {
    [
        Tag::PlatformTailNumber,
        Tag::PlatformCallSign,
        Tag::PlatformDesignation,
        Tag::MissionId,
    ]
    .into_iter()
    .filter_map(|t| p.text(t))
    .map(str::trim)
    .find(|s| !s.is_empty())
}

/// The platform UID used by [`platform`] and [`spi`].
pub fn platform_uid(p: &Packet<'_>, opts: &Options) -> String {
    match &opts.uid {
        Some(u) => u.clone(),
        None => format!("fmv-{}", name(p).unwrap_or("UAS")),
    }
}

/// The platform CoT type for `opts`.
pub fn platform_type(opts: &Options) -> String {
    format!("a-{}-A-M-F-Q", opts.affiliation)
}

fn time_us(p: &Packet<'_>) -> u64 {
    p.timestamp().unwrap_or_else(crate::now_us)
}

/// The aircraft: sensor position (tags 13, 14; height above ellipsoid from
/// tag 104 or 75, else the MSL altitude of tag 15), course and ground speed,
/// and the camera's line of sight (`<sensor>`): azimuth = platform heading
/// + sensor relative azimuth, field of view, slant range.
pub fn platform(p: &Packet<'_>, opts: &Options) -> Option<Event> {
    let uid = platform_uid(p, opts);
    let pos = p.sensor_position()?;
    let mut e = Event::new(uid, platform_type(opts), time_us(p), opts.stale_secs);
    let (hae, altsrc) = match p.sensor_hae() {
        Some(h) => (h, "GPS"),
        None => (pos.height.unwrap_or(0.0), "MSL"),
    };
    e.point = Point::new(pos.lat, pos.lon, hae);
    if !e.point.is_valid() {
        return None;
    }
    e.precision_location = Some(("GPS".into(), altsrc.into()));
    e.callsign = Some(name(p).map_or_else(|| e.uid.clone(), str::to_string));
    let heading = p.platform_heading();
    let course = p.float(Tag::PlatformCourseAngle).or(heading);
    if let (Some(c), Some(s)) = (course, p.float(Tag::PlatformGroundSpeed)) {
        e.track = Some(Track {
            course: c,
            speed: s,
        });
    }
    let rel_az = p.float(Tag::SensorRelativeAzimuthAngle);
    let sensor = Sensor {
        azimuth: heading.zip(rel_az).map(|(h, a)| h + a),
        elevation: p.float(Tag::SensorRelativeElevationAngle),
        roll: p.float(Tag::SensorRelativeRollAngle),
        fov: p.float(Tag::SensorHorizontalFieldOfView),
        vfov: p.float(Tag::SensorVerticalFieldOfView),
        range: p.float(Tag::SlantRange),
        model: p.text(Tag::ImageSourceSensor).map(str::to_string),
        kind: None,
    };
    if sensor.fov.is_some() || sensor.azimuth.is_some() {
        e.sensor = Some(sensor);
    }
    e.video_url = opts.video_url.clone();
    let mut r = String::from("MISB ST 0601.");
    for (t, label) in [
        (Tag::MissionId, "Mission"),
        (Tag::PlatformDesignation, "Platform"),
        (Tag::ImageSourceSensor, "Sensor"),
    ] {
        if let Some(v) = p.text(t).filter(|s| !s.trim().is_empty()) {
            let _ = write!(r, " {label}: {}.", v.trim());
        }
    }
    e.remarks = Some(r);
    Some(e)
}

/// The sensor point of interest (`b-m-p-s-p-i`): the frame center (tags 23,
/// 24; height from tag 78, else tag 25), linked to the platform.
pub fn spi(p: &Packet<'_>, opts: &Options) -> Option<Event> {
    let puid = platform_uid(p, opts);
    let fc = p.frame_center()?;
    let mut e = Event::new(
        format!("{puid}.SPI1"),
        "b-m-p-s-p-i",
        time_us(p),
        opts.stale_secs,
    );
    e.how = "m-i".into();
    let hae = p
        .float(Tag::FrameCenterHeightAboveEllipsoid)
        .or(fc.height)
        .unwrap_or(0.0);
    e.point = Point::new(fc.lat, fc.lon, hae);
    if !e.point.is_valid() {
        return None;
    }
    if let Some(ce) = p.float(Tag::TargetErrorEstimateCe90) {
        // CE90 to 1-sigma circular error
        e.point.ce = ce / 2.146;
    }
    e.callsign = Some(format!("{} SPI", name(p).unwrap_or(&puid)));
    e.links.push(Link {
        uid: puid,
        kind: platform_type(opts),
        relation: "p-p".into(),
    });
    Some(e)
}

/// Platform and SPI events for one packet (platform first).
pub fn events(p: &Packet<'_>, opts: &Options) -> Vec<Event> {
    platform(p, opts).into_iter().chain(spi(p, opts)).collect()
}
