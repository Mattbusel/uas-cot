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
//!
//! With a [`remoteid::Tracker`], [`tracked`] also writes the transmitter's
//! trust verdict (spoofed, suspicious, plausible, authenticated) and its
//! reasons into the aircraft's remarks, so a TAK user sees why a marker
//! should not be believed.

use crate::{Event, Point, Track, UNKNOWN_ERROR};
use remoteid::{Assessment, OperatorLocationType, TrackedUas, UaType, UasData, Verdict};
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

/// The trust verdict as one remarks sentence, for example
/// `Trust: spoofed (teleport, known_spoofer; tool: jjshoots/RemoteIDSpoofer).`
/// or `Trust: plausible.` when nothing contradicts the broadcast.
pub fn trust_remarks(a: &Assessment) -> String {
    let mut r = format!("Trust: {}", a.verdict.name());
    let reasons: Vec<&str> = a.reasons.iter().map(|x| x.name()).collect();
    if !reasons.is_empty() || a.spoofer.is_some() {
        r.push_str(" (");
        r.push_str(&reasons.join(", "));
        if let Some(t) = a.spoofer {
            if !reasons.is_empty() {
                r.push_str("; ");
            }
            let _ = write!(r, "tool: {}", t.name());
        }
        r.push(')');
    }
    r.push('.');
    r
}

/// [`events`] for a transmitter kept by [`remoteid::Tracker`], with
/// [`trust_remarks`] for its latest assessment appended to the aircraft's
/// remarks. Markers and UIDs are the same as from [`events`]; the
/// affiliation stays `opts.affiliation`, so a caller who wants spoofed
/// aircraft drawn as suspect sets it from `uas.assessment.verdict`.
///
/// ```
/// use uas_cot::remoteid::{BasicId, IdType, Location, Message, Tracker, Transmitter, UaType};
/// use uas_cot::rid::{self, Options};
///
/// let mut tracker = Tracker::default();
/// let tx = Transmitter::Ble([0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]);
/// let id = BasicId::new(IdType::SerialNumber, UaType::HelicopterOrMultirotor, "1596F35074529ABCDEFG").unwrap();
/// let mut loc = Location::default();
/// loc.latitude = 45.5470818;
/// loc.longitude = -122.9668346;
/// tracker.ingest(tx, 1, &Message::BasicId(id).encode().unwrap(), 1_000);
/// let uas = tracker.ingest(tx, 2, &Message::Location(loc).encode().unwrap(), 1_100);
///
/// let events = rid::tracked(uas, None, 1_100_000, &Options::default());
/// assert!(events[0].remarks.as_deref().unwrap().ends_with("Trust: plausible."));
/// ```
pub fn tracked(
    uas: &TrackedUas,
    fallback_id: Option<&str>,
    now_us: u64,
    opts: &Options,
) -> Vec<Event> {
    let mut out = events(&uas.data, fallback_id, now_us, opts);
    if let Some(air) = out.first_mut().filter(|e| e.kind.starts_with("a-")) {
        let t = trust_remarks(&uas.assessment);
        air.remarks = Some(match air.remarks.take() {
            Some(r) => format!("{r} {t}"),
            None => t,
        });
    }
    out
}

/// The ground marker from the System message, labelled by what the position
/// is. Remote ID sends one ground position and a type that says whether it
/// is the operator's live GNSS fix, a fixed operator location, or the
/// takeoff point. [`operator`] calls all three "Operator of ...", which
/// sends a security team to where the aircraft left from, not to where the
/// pilot stands. This one keeps them apart:
///
/// | Location type | UID | Callsign |
/// |---|---|---|
/// | live GNSS | `<prefix>op-<id>` | `Operator of <id>` |
/// | fixed | `<prefix>op-<id>` | `Operator of <id> (fixed location)` |
/// | takeoff | `<prefix>takeoff-<id>` | `Takeoff point of <id>` |
///
/// A takeoff marker is a separate entity on the TAK map: if the drone later
/// switches to live operator location, the operator marker appears next to
/// it instead of moving the takeoff marker.
///
/// ```
/// use uas_cot::remoteid::{BasicId, IdType, Message, OperatorLocationType, System, UaType, UasData};
/// use uas_cot::rid::{self, Options};
///
/// let mut uas = UasData::default();
/// let id = BasicId::new(IdType::SerialNumber, UaType::HelicopterOrMultirotor, "1596F35074529ABCDEFG").unwrap();
/// uas.update(&Message::BasicId(id));
/// let mut sys = System::default();
/// sys.operator_location_type = OperatorLocationType::Takeoff;
/// sys.operator_latitude = 45.5470;
/// sys.operator_longitude = -122.9668;
/// uas.update(&Message::System(sys));
///
/// let e = rid::operator_or_takeoff(&uas, None, 1_700_000_000_000_000, &Options::default()).unwrap();
/// assert_eq!(e.uid, "rid-takeoff-1596F35074529ABCDEFG");
/// assert_eq!(e.callsign.as_deref(), Some("Takeoff point of 1596F35074529ABCDEFG"));
/// ```
pub fn operator_or_takeoff(
    data: &UasData,
    fallback_id: Option<&str>,
    now_us: u64,
    opts: &Options,
) -> Option<Event> {
    let mut e = operator(data, fallback_id, now_us, opts)?;
    let id = id_or(data, fallback_id)?;
    let sys = data.system.as_ref()?;
    let op = data
        .operator_id
        .as_ref()
        .and_then(|o| o.operator_id_str())
        .filter(|s| !s.is_empty());
    match sys.operator_location_type {
        OperatorLocationType::Takeoff => {
            e.uid = format!("{}takeoff-{id}", opts.uid_prefix);
            e.callsign = Some(match op {
                Some(op) => format!("Takeoff point of {id} (operator {op})"),
                None => format!("Takeoff point of {id}"),
            });
            e.remarks = Some(
                "Remote ID takeoff point. Not a live operator position: the aircraft \
                 does not broadcast where its pilot is."
                    .into(),
            );
        }
        OperatorLocationType::Fixed => {
            e.callsign = Some(match op {
                Some(op) => format!("{op} (operator of {id}, fixed location)"),
                None => format!("Operator of {id} (fixed location)"),
            });
            e.remarks =
                Some("Remote ID operator location (fixed, entered by the operator).".into());
        }
        OperatorLocationType::LiveGnss => {
            e.remarks = Some("Remote ID operator location (live GNSS).".into());
        }
        OperatorLocationType::Other(v) => {
            e.remarks = Some(format!(
                "Remote ID operator location (reserved location type {v})."
            ));
        }
    }
    Some(e)
}

/// How a tracked transmitter's affiliation (the second letter of the CoT
/// type, `a-?-...`) is chosen from its trust verdict and from a list of
/// your own aircraft.
///
/// The order matters and is fixed:
///
/// 1. A `spoofed` verdict gives [`spoofed`](Self::spoofed) (default `'s'`,
///    suspect), and a `suspicious` verdict gives
///    [`suspicious`](Self::suspicious) (default: no change). This comes
///    before the friendly list on purpose: a spoofer that copies the serial
///    number of one of your drones must not turn into a friendly marker.
/// 2. A Basic ID or Operator ID in [`friendly_ids`](Self::friendly_ids)
///    gives [`friendly`](Self::friendly) (default `'f'`), unless the
///    verdict is `spoofed` or `suspicious`. With
///    [`friendly_requires_authenticated`](Self::friendly_requires_authenticated)
///    only an `authenticated` (DRIP signed) transmitter qualifies.
/// 3. An `authenticated` verdict gives
///    [`authenticated`](Self::authenticated) (default: no change).
/// 4. Otherwise the default passed in ([`Options::affiliation`]).
///
/// ```
/// use uas_cot::rid::AffiliationRules;
///
/// let rules = AffiliationRules::default()
///     .friendly_id("1596F35074529ABCDEFG")
///     .suspicious_as('s')
///     .spoofed_as('h');
/// assert_eq!(rules.friendly_ids, ["1596F35074529ABCDEFG"]);
/// ```
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct AffiliationRules {
    /// Basic IDs (serial numbers, registrations, session IDs) and Operator
    /// IDs of your own aircraft. Matched exactly, after trimming spaces.
    pub friendly_ids: Vec<String>,
    /// Affiliation for a match in `friendly_ids` (default `'f'`).
    pub friendly: char,
    /// Only an `authenticated` transmitter counts as friendly (default false:
    /// most drones do not sign their broadcasts yet).
    pub friendly_requires_authenticated: bool,
    /// Affiliation for a `spoofed` verdict (default `Some('s')`, suspect).
    pub spoofed: Option<char>,
    /// Affiliation for a `suspicious` verdict (default `None`: unchanged).
    pub suspicious: Option<char>,
    /// Affiliation for an `authenticated` verdict that is not on the
    /// friendly list (default `None`: unchanged).
    pub authenticated: Option<char>,
}

impl Default for AffiliationRules {
    fn default() -> Self {
        AffiliationRules {
            friendly_ids: Vec::new(),
            friendly: 'f',
            friendly_requires_authenticated: false,
            spoofed: Some('s'),
            suspicious: None,
            authenticated: None,
        }
    }
}

impl AffiliationRules {
    /// Adds one of your aircraft (Basic ID or Operator ID).
    pub fn friendly_id(mut self, id: impl Into<String>) -> Self {
        self.friendly_ids.push(id.into().trim().to_string());
        self
    }

    /// Sets [`friendly`](Self::friendly).
    pub fn friendly_as(mut self, affiliation: char) -> Self {
        self.friendly = affiliation;
        self
    }

    /// Sets [`friendly_requires_authenticated`](Self::friendly_requires_authenticated).
    pub fn require_authenticated_friendly(mut self, yes: bool) -> Self {
        self.friendly_requires_authenticated = yes;
        self
    }

    /// Sets [`spoofed`](Self::spoofed).
    pub fn spoofed_as(mut self, affiliation: char) -> Self {
        self.spoofed = Some(affiliation);
        self
    }

    /// Sets [`suspicious`](Self::suspicious).
    pub fn suspicious_as(mut self, affiliation: char) -> Self {
        self.suspicious = Some(affiliation);
        self
    }

    /// Sets [`authenticated`](Self::authenticated).
    pub fn authenticated_as(mut self, affiliation: char) -> Self {
        self.authenticated = Some(affiliation);
        self
    }

    /// Clears every verdict rule, so a verdict never changes the
    /// affiliation; only the friendly list applies (still not to a
    /// `spoofed` or `suspicious` transmitter).
    pub fn no_verdict_rules(mut self) -> Self {
        self.spoofed = None;
        self.suspicious = None;
        self.authenticated = None;
        self
    }

    fn is_friendly(&self, data: &UasData) -> bool {
        if self.friendly_ids.is_empty() {
            return false;
        }
        let listed = |s: &str| {
            let s = s.trim();
            !s.is_empty() && self.friendly_ids.iter().any(|f| f == s)
        };
        data.basic_id
            .iter()
            .flatten()
            .filter_map(|b| b.uas_id_str())
            .any(listed)
            || data
                .operator_id
                .as_ref()
                .and_then(|o| o.operator_id_str())
                .is_some_and(listed)
    }

    /// The affiliation for `uas`, or `default` when no rule applies.
    pub fn affiliation(&self, uas: &TrackedUas, default: char) -> char {
        let verdict = uas.assessment.verdict;
        let trusted = match verdict {
            Verdict::Spoofed => return self.spoofed.unwrap_or(default),
            Verdict::Suspicious => return self.suspicious.unwrap_or(default),
            Verdict::Authenticated => true,
            Verdict::Plausible => !self.friendly_requires_authenticated,
        };
        if trusted && self.is_friendly(&uas.data) {
            return self.friendly;
        }
        match (verdict, self.authenticated) {
            (Verdict::Authenticated, Some(c)) => c,
            _ => default,
        }
    }
}

/// Like [`tracked`], with two differences: the aircraft's affiliation comes
/// from `rules` (see [`AffiliationRules`]), and the ground marker comes from
/// [`operator_or_takeoff`], so a takeoff point is not labelled as the
/// operator. Aircraft first, then the ground marker.
///
/// ```
/// use uas_cot::remoteid::{BasicId, IdType, Location, Message, Tracker, Transmitter, UaType};
/// use uas_cot::rid::{self, AffiliationRules, Options};
///
/// let mut tracker = Tracker::default();
/// let tx = Transmitter::Ble([0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]);
/// let id = BasicId::new(IdType::SerialNumber, UaType::HelicopterOrMultirotor, "1596F35074529ABCDEFG").unwrap();
/// let mut loc = Location::default();
/// loc.latitude = 45.5470818;
/// loc.longitude = -122.9668346;
/// tracker.ingest(tx, 1, &Message::BasicId(id).encode().unwrap(), 1_000);
/// let uas = tracker.ingest(tx, 2, &Message::Location(loc).encode().unwrap(), 1_100);
///
/// let rules = AffiliationRules::default().friendly_id("1596F35074529ABCDEFG");
/// let events = rid::tracked_with(uas, None, 1_100_000, &Options::default(), &rules);
/// assert_eq!(events[0].kind, "a-f-A-M-F-Q");
/// ```
pub fn tracked_with(
    uas: &TrackedUas,
    fallback_id: Option<&str>,
    now_us: u64,
    opts: &Options,
    rules: &AffiliationRules,
) -> Vec<Event> {
    let air_opts = Options {
        affiliation: rules.affiliation(uas, opts.affiliation),
        ..opts.clone()
    };
    let mut out = Vec::with_capacity(2);
    if let Some(mut air) = aircraft(&uas.data, fallback_id, now_us, &air_opts) {
        let t = trust_remarks(&uas.assessment);
        air.remarks = Some(match air.remarks.take() {
            Some(r) => format!("{r} {t}"),
            None => t,
        });
        out.push(air);
    }
    out.extend(operator_or_takeoff(&uas.data, fallback_id, now_us, opts));
    out
}

#[cfg(test)]
#[allow(clippy::field_reassign_with_default)] // remoteid's message structs are non_exhaustive
mod tests {
    use super::*;
    use remoteid::{BasicId, IdType, Location, Message, OperatorId, System, Tracker, Transmitter};

    const SERIAL: &str = "1596F35074529ABCDEFG";

    fn tracked_uas(loc_type: OperatorLocationType, operator: Option<&str>) -> TrackedUas {
        let mut tracker = Tracker::default();
        let tx = Transmitter::Ble([1, 2, 3, 4, 5, 6]);
        let id =
            BasicId::new(IdType::SerialNumber, UaType::HelicopterOrMultirotor, SERIAL).unwrap();
        let mut loc = Location::default();
        loc.latitude = 45.5470818;
        loc.longitude = -122.9668346;
        let mut sys = System::default();
        sys.operator_location_type = loc_type;
        sys.operator_latitude = 45.5443876;
        sys.operator_longitude = -122.9726866;
        let mut msgs = vec![
            Message::BasicId(id),
            Message::Location(loc),
            Message::System(sys),
        ];
        if let Some(op) = operator {
            msgs.push(Message::OperatorId(OperatorId::new(op).unwrap()));
        }
        let mut last = None;
        for (i, m) in msgs.iter().enumerate() {
            let t = 1_000 + i as u64 * 100;
            last = Some(tracker.ingest(tx, i as u8, &m.encode().unwrap(), t).clone());
        }
        last.unwrap()
    }

    fn with_verdict(mut uas: TrackedUas, v: Verdict) -> TrackedUas {
        uas.assessment.verdict = v;
        uas
    }

    #[test]
    fn takeoff_is_its_own_entity() {
        let uas = tracked_uas(OperatorLocationType::Takeoff, Some("GBR-OP-123ABCD"));
        let e = operator_or_takeoff(&uas.data, None, 0, &Options::default()).unwrap();
        assert_eq!(e.uid, format!("rid-takeoff-{SERIAL}"));
        assert_eq!(
            e.callsign.as_deref(),
            Some(format!("Takeoff point of {SERIAL} (operator GBR-OP-123ABCD)").as_str())
        );
        assert!(e
            .remarks
            .as_deref()
            .unwrap()
            .contains("Not a live operator position"));
        assert_eq!(e.kind, "b-m-p-s-m");
        // The old function still calls it the operator, unchanged.
        let old = operator(&uas.data, None, 0, &Options::default()).unwrap();
        assert_eq!(old.uid, format!("rid-op-{SERIAL}"));
    }

    #[test]
    fn live_and_fixed_operator_positions() {
        let live = tracked_uas(OperatorLocationType::LiveGnss, None);
        let e = operator_or_takeoff(&live.data, None, 0, &Options::default()).unwrap();
        assert_eq!(e.uid, format!("rid-op-{SERIAL}"));
        assert_eq!(
            e.callsign.as_deref(),
            Some(format!("Operator of {SERIAL}").as_str())
        );
        assert!(e.remarks.as_deref().unwrap().contains("live GNSS"));

        let fixed = tracked_uas(OperatorLocationType::Fixed, None);
        let e = operator_or_takeoff(&fixed.data, None, 0, &Options::default()).unwrap();
        assert_eq!(e.uid, format!("rid-op-{SERIAL}"));
        assert!(e.callsign.as_deref().unwrap().ends_with("(fixed location)"));
    }

    #[test]
    fn spoofed_never_becomes_friendly() {
        let rules = AffiliationRules::default().friendly_id(SERIAL);
        let base = tracked_uas(OperatorLocationType::Takeoff, None);
        assert_eq!(
            rules.affiliation(&with_verdict(base.clone(), Verdict::Plausible), 'u'),
            'f'
        );
        assert_eq!(
            rules.affiliation(&with_verdict(base.clone(), Verdict::Spoofed), 'u'),
            's'
        );
        // suspicious with no suspicious rule: the default, never the friendly letter
        assert_eq!(
            rules.affiliation(&with_verdict(base.clone(), Verdict::Suspicious), 'u'),
            'u'
        );
        let strict = rules.clone().suspicious_as('s').spoofed_as('h');
        assert_eq!(
            strict.affiliation(&with_verdict(base.clone(), Verdict::Suspicious), 'u'),
            's'
        );
        assert_eq!(
            strict.affiliation(&with_verdict(base.clone(), Verdict::Spoofed), 'u'),
            'h'
        );
        let none = rules.clone().no_verdict_rules();
        assert_eq!(
            none.affiliation(&with_verdict(base, Verdict::Spoofed), 'u'),
            'u'
        );
    }

    #[test]
    fn friendly_by_operator_id_and_auth_requirement() {
        let uas = tracked_uas(OperatorLocationType::LiveGnss, Some("FIN87astrdge12k8"));
        let rules = AffiliationRules::default().friendly_id(" FIN87astrdge12k8 ");
        assert_eq!(rules.affiliation(&uas, 'u'), 'f');
        let strict = rules.clone().require_authenticated_friendly(true);
        assert_eq!(
            strict.affiliation(&with_verdict(uas.clone(), Verdict::Plausible), 'u'),
            'u'
        );
        assert_eq!(
            strict.affiliation(&with_verdict(uas.clone(), Verdict::Authenticated), 'u'),
            'f'
        );
        let other = AffiliationRules::default().authenticated_as('n');
        assert_eq!(
            other.affiliation(&with_verdict(uas.clone(), Verdict::Authenticated), 'u'),
            'n'
        );
        assert_eq!(other.affiliation(&uas, 'u'), 'u');
    }

    #[test]
    fn tracked_with_sets_type_and_ground_marker() {
        let uas = with_verdict(
            tracked_uas(OperatorLocationType::Takeoff, None),
            Verdict::Spoofed,
        );
        let ev = tracked_with(
            &uas,
            None,
            0,
            &Options::default(),
            &AffiliationRules::default(),
        );
        assert_eq!(ev.len(), 2);
        assert_eq!(ev[0].kind, "a-s-A-M-F-Q");
        assert!(ev[0].remarks.as_deref().unwrap().contains("Trust: spoofed"));
        assert_eq!(ev[1].uid, format!("rid-takeoff-{SERIAL}"));
        // tracked() is unchanged: opts.affiliation, operator labelling
        let old = tracked(&uas, None, 0, &Options::default());
        assert_eq!(old[0].kind, "a-u-A-M-F-Q");
        assert_eq!(old[1].uid, format!("rid-op-{SERIAL}"));
    }
}
