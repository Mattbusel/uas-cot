//! MAVLink-M friendly position reports to CoT events.

use super::dialect::{Dialect, Message, Value};
use super::{military, scope, PARTICIPANT_POSITION};
use crate::{Event, Point, Track};

/// How MAVLink-M messages become events.
#[derive(Clone, Debug, PartialEq)]
pub struct Options {
    /// Seconds until stale (default 30).
    pub stale_secs: u64,
    /// UID prefix of the events this module makes (default `"mavlink-m-"`).
    pub uid_prefix: String,
    /// Drop messages that fail [`super::validate::check_payload`] (default
    /// true). The dialect requires receivers to reject undefined values of
    /// its protected enums rather than guess.
    pub reject_invalid: bool,
}

impl Default for Options {
    fn default() -> Self {
        Options {
            stale_secs: 30,
            uid_prefix: "mavlink-m-".into(),
            reject_invalid: true,
        }
    }
}

/// CoT affiliation letter for a `MAVLINK_M_STANAG_IDENTITY` value
/// (pending, unknown, assumed friend, friend, neutral, suspect, hostile).
pub fn affiliation_from_identity(v: u8) -> char {
    match v {
        0 => 'p',
        1 => 'u',
        2 => 'a',
        3 => 'f',
        4 => 'n',
        5 => 's',
        6 => 'h',
        _ => 'u',
    }
}

/// CoT dimension for a `MAVLINK_M_PPLI_TYPE`.
pub fn ppli_dimension(t: u8) -> Option<&'static str> {
    Some(match t {
        1 => "A",
        2 => "G",
        3 => "S",
        4 => "U",
        _ => return None,
    })
}

fn get(m: &Message, p: &[u8], name: &str) -> Value {
    m.get(p, name).unwrap_or(Value::Uint(0))
}
fn u(m: &Message, p: &[u8], name: &str) -> u64 {
    get(m, p, name).as_u64().unwrap_or(0)
}
fn i(m: &Message, p: &[u8], name: &str) -> i64 {
    get(m, p, name).as_i64().unwrap_or(0)
}
fn fl(m: &Message, p: &[u8], name: &str) -> f64 {
    get(m, p, name).as_f64().unwrap_or(f64::NAN)
}
fn text(m: &Message, p: &[u8], name: &str) -> String {
    get(m, p, name).as_str().unwrap_or("").trim().to_string()
}

fn track_from_ned(vn: f64, ve: f64) -> Option<Track> {
    if vn.is_finite() && ve.is_finite() {
        Some(Track {
            course: ve.atan2(vn).to_degrees().rem_euclid(360.0),
            speed: vn.hypot(ve),
        })
    } else {
        None
    }
}

/// Turns MAVLink-M friendly position reports into CoT events.
///
/// Translated: `PARTICIPANT_POSITION` (53003) only, the list in
/// [`scope::TRANSLATED`]. Vehicle telemetry (`HEARTBEAT`,
/// `GLOBAL_POSITION_INT`) is [`crate::mavlink`]'s job.
///
/// Blocked: every target, track, handover, fires, munition, arming,
/// weapon and engagement message ([`scope::is_blocked`]) is dropped before
/// its payload is read, never produces an event, and is counted in
/// [`Gateway::blocked`]. Every other MAVLink-M message is ignored.
#[derive(Debug)]
pub struct Gateway {
    opts: Options,
    dialect: &'static Dialect,
    blocked: u64,
}

impl Default for Gateway {
    fn default() -> Self {
        Gateway::new(Options::default())
    }
}

impl Gateway {
    /// A gateway using the embedded MAVLink-M definition.
    pub fn new(opts: Options) -> Self {
        Gateway {
            opts,
            dialect: military(),
            blocked: 0,
        }
    }

    /// How many out-of-scope messages this gateway has refused.
    pub fn blocked(&self) -> u64 {
        self.blocked
    }

    /// Handles one message from system `sys_id` and returns the event it
    /// produces, if any. `now_us` is the receive time (or the log time when
    /// replaying). Out-of-scope messages always return `None`.
    pub fn ingest(
        &mut self,
        sys_id: u8,
        msg_id: u32,
        payload: &[u8],
        now_us: u64,
    ) -> Option<Event> {
        let name = self.dialect.message(msg_id).map(|m| m.name.as_str());
        if scope::is_blocked(msg_id, name.unwrap_or("")) {
            self.blocked += 1;
            return None;
        }
        if !scope::is_translated(msg_id) {
            return None;
        }
        let m = self.dialect.message(msg_id)?;
        if self.opts.reject_invalid
            && !super::validate::check_payload(self.dialect, msg_id, payload, true).is_ok()
        {
            return None;
        }
        match msg_id {
            PARTICIPANT_POSITION => self.participant(m, sys_id, payload, now_us),
            _ => None,
        }
    }

    fn label(&self, en: &str, v: u64) -> String {
        self.dialect
            .label(en, v)
            .unwrap_or_else(|| format!("undefined value {v}"))
    }

    fn participant(&mut self, m: &Message, sys_id: u8, p: &[u8], now_us: u64) -> Option<Event> {
        let lat = i(m, p, "lat");
        let lon = i(m, p, "lon");
        if lat == 0 && lon == 0 {
            return None;
        }
        let point = Point::new(lat as f64 * 1e-7, lon as f64 * 1e-7, fl(m, p, "alt"));
        if !point.is_valid() {
            return None;
        }
        let origin = u(m, p, "origin_sysid") as u8;
        let origin = if origin == 0 { sys_id } else { origin };
        let mut e = Event::new(
            format!("{}ppli-{origin}", self.opts.uid_prefix),
            "a-u-X",
            now_us,
            self.opts.stale_secs,
        );
        let t = u(m, p, "time_usec");
        if t != 0 && t <= now_us {
            e.start_us = t;
        }
        e.point = point;
        if !e.point.hae.is_finite() {
            e.point.hae = 0.0;
        }
        e.precision_location = Some(("GPS".into(), "MSL".into()));
        let (vn, ve) = (fl(m, p, "vx"), fl(m, p, "vy"));
        let course = fl(m, p, "course");
        e.track = match track_from_ned(vn, ve) {
            Some(mut t) => {
                if course.is_finite() {
                    t.course = course;
                }
                Some(t)
            }
            None => None,
        };
        let cs = text(m, p, "callsign");
        e.callsign = Some(if cs.is_empty() {
            format!("PPLI {origin}")
        } else {
            cs
        });
        let aff = affiliation_from_identity(u(m, p, "stanag_identity") as u8);
        let ppli = u(m, p, "ppli_type") as u8;
        e.kind = format!("a-{aff}-{}", ppli_dimension(ppli).unwrap_or("X"));
        let mut r = format!(
            "MAVLink-M PARTICIPANT_POSITION from system {origin}. Type: {}. Identity: {}.",
            self.label("MAVLINK_M_PPLI_TYPE", ppli as u64),
            self.label("MAVLINK_M_STANAG_IDENTITY", u(m, p, "stanag_identity"))
        );
        let etn = text(m, p, "external_track_number");
        if !etn.is_empty() {
            r.push_str(&format!(
                " Network participant number {etn} ({}).",
                self.label(
                    "MAVLINK_M_TRACK_NUMBER_TYPE",
                    u(m, p, "external_track_type")
                )
            ));
        }
        e.remarks = Some(r);
        Some(e)
    }
}
