//! MAVLink-M track messages to CoT events.

use super::dialect::{Dialect, Message, Value};
use super::{military, PARTICIPANT_POSITION, TARGET, TARGET_HANDOVER, TRACK_IDENTITY};
use crate::{Event, Link, Point, Track, UNKNOWN_ERROR};
use std::collections::HashMap;

/// How MAVLink-M messages become events.
#[derive(Clone, Debug, PartialEq)]
pub struct Options {
    /// Seconds until stale when the message gives no validity (default 30).
    pub stale_secs: u64,
    /// UID prefix of the events this module makes (default `"mavlink-m-"`).
    pub uid_prefix: String,
    /// UID prefix of the reporting platform, linked as the producer of each
    /// track (default `"mav-"`, the prefix [`crate::mavlink`] uses, so the
    /// link points at the vehicle marker of the same system id).
    pub platform_uid_prefix: String,
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
            platform_uid_prefix: "mav-".into(),
            reject_invalid: true,
        }
    }
}

/// Identity of a track, from `TRACK_IDENTITY`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Identity {
    /// `stanag_identity` (MIL-STD-2525D standard identity code).
    pub stanag_identity: u8,
    /// `target_force`.
    pub target_force: u8,
    /// `target_class`.
    pub target_class: u8,
    /// `environment`.
    pub environment: u8,
    /// `sidc_context`: 0 reality, 1 exercise, 2 simulation.
    pub sidc_context: u8,
    /// `id_method`.
    pub id_method: u8,
    /// `pid_status`.
    pub pid_status: u8,
    /// `id_confidence`, 0 to 1 (NaN if not given).
    pub id_confidence: f64,
    /// `external_track_number`.
    pub external_track_number: String,
    /// `external_track_type`.
    pub external_track_type: u8,
    /// `origin_sysid`.
    pub origin_sysid: u8,
}

/// Formats a 16-byte track UID as a UUID string.
pub fn uuid_string(b: &[u8; 16]) -> String {
    let h: String = b.iter().map(|x| format!("{x:02x}")).collect();
    format!(
        "{}-{}-{}-{}-{}",
        &h[0..8],
        &h[8..12],
        &h[12..16],
        &h[16..20],
        &h[20..32]
    )
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

/// CoT affiliation letter for a `MAVLINK_M_TARGET_FORCE` value.
pub fn affiliation_from_force(v: u8) -> char {
    match v {
        10 => 'n',
        20 => 'f',
        30 => 'h',
        _ => 'u',
    }
}

/// The CoT battle dimension and MIL-STD-2525B function for a
/// `MAVLINK_M_TARGET_CLASS`, for example `"G-E-V-A-T"` for a main battle
/// tank. `None` for classes with no fixed dimension (unknown, emitters,
/// other); those use the domain or environment instead. Every code here is
/// a function ID from the MIL-STD-2525B change 2 tables (checked in the
/// tests). `civil` picks civil instead of military aircraft.
pub fn class_type(class: u8, civil: bool) -> Option<&'static str> {
    Some(match class {
        1 => "G-E-V",
        2 => "G-E-V-A-T",
        3 => "G-E-V-A-I",
        4 => "G-E-V-A-A",
        5 | 6 => "G-E-V-U",
        7 | 8 => "G-E-W-H",
        9 => "G-E-W-X",
        10 => "G-E-W-M-A",
        11 => "G-E-S-R",
        12 | 15 => "G-E-V",
        13 => "G-E-V-A-C",
        14 => "G-E-V-E",
        50..=53 => "G-U-C-I",
        80 => "A",
        81 if civil => "A-C-F",
        81 => "A-M-F",
        82 if civil => "A-C-H",
        82 => "A-M-H",
        83 => "A-M-F-Q",
        84 => "A-M-H-Q",
        85 | 87 => "A-W",
        86 => "A-W-M",
        120 => "S",
        121 => "S-C",
        122 => "S-C-P",
        123 => "S-C-A-L-C",
        124 => "S-N",
        125 => "S-X",
        126 => "S-C-U",
        160 => "U",
        161 => "U-S",
        162 => "U-S-U",
        163 => "U-W-M",
        190..=193 => "G-I-T",
        194 | 195 => "G-I",
        196 => "G-I-U-E",
        197 => "G-I-U-T",
        240 | 242 => "P",
        241 => "P-S",
        243 => "P-L",
        _ => return None,
    })
}

/// CoT dimension for a `MAVLINK_M_TARGET_DOMAIN`.
pub fn domain_dimension(domain: u8) -> Option<&'static str> {
    Some(match domain {
        1 | 8 => "G",
        2 => "A",
        3 => "S",
        4 => "U",
        5 => "P",
        _ => return None,
    })
}

/// CoT dimension for a `MAVLINK_M_ENVIRONMENT` (surface is land or sea, so
/// it gives none).
pub fn environment_dimension(env: u8) -> Option<&'static str> {
    Some(match env {
        1 => "P",
        2 => "A",
        4 => "U",
        5 => "G",
        _ => return None,
    })
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
fn uid16(m: &Message, p: &[u8], name: &str) -> Option<[u8; 16]> {
    let b = get(m, p, name).as_bytes()?;
    let a: [u8; 16] = b.try_into().ok()?;
    if a.iter().all(|&x| x == 0) {
        None
    } else {
        Some(a)
    }
}

/// 1-sigma error from a variance; zero, negative or non-finite means
/// unknown (a zero-filled field must not claim a perfect fix).
fn sigma(var: f64) -> f64 {
    if var.is_finite() && var > 0.0 {
        var.sqrt()
    } else {
        UNKNOWN_ERROR
    }
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

/// Last kinematic report of a track, kept so an identity update can
/// re-issue the marker with its new type.
#[derive(Clone, Debug)]
struct Kinematics {
    event: Event,
    class: u8,
    force: u8,
    domain_dim: Option<&'static str>,
}

/// Turns a stream of MAVLink-M messages into CoT events.
///
/// Translated: `TRACK_IDENTITY` (53000), `TARGET_HANDOVER` (53002),
/// `PARTICIPANT_POSITION` (53003) and `TARGET` (53010). Identity from
/// `TRACK_IDENTITY` is joined to `TARGET_HANDOVER` reports by `track_uid`.
///
/// Fields that are deliberately never translated: `authorization` tokens,
/// `prf_code`, target package endpoints and paths, `cep_desired` /
/// `cep_max`, `tle_category`, `dmpi_reference_kind` and
/// `restricted_target_flags`. The output is a map picture (who and what is
/// where), not engagement data. Every other MAVLink-M message is ignored.
#[derive(Debug)]
pub struct Gateway {
    opts: Options,
    dialect: &'static Dialect,
    identities: HashMap<[u8; 16], Identity>,
    tracks: HashMap<[u8; 16], Kinematics>,
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
            identities: HashMap::new(),
            tracks: HashMap::new(),
        }
    }

    /// The identity last reported for `track_uid`.
    pub fn identity(&self, track_uid: &[u8; 16]) -> Option<&Identity> {
        self.identities.get(track_uid)
    }

    /// Handles one message from system `sys_id` and returns the event it
    /// produces, if any. `now_us` is the receive time (or the log time when
    /// replaying).
    pub fn ingest(
        &mut self,
        sys_id: u8,
        msg_id: u32,
        payload: &[u8],
        now_us: u64,
    ) -> Option<Event> {
        let m = self.dialect.message(msg_id)?;
        if !matches!(
            msg_id,
            TRACK_IDENTITY | TARGET_HANDOVER | PARTICIPANT_POSITION | TARGET
        ) {
            return None;
        }
        if self.opts.reject_invalid
            && !super::validate::check_payload(self.dialect, msg_id, payload, true).is_ok()
        {
            return None;
        }
        match msg_id {
            TRACK_IDENTITY => self.track_identity(m, payload),
            TARGET_HANDOVER => self.handover(m, sys_id, payload, now_us),
            PARTICIPANT_POSITION => self.participant(m, sys_id, payload, now_us),
            TARGET => self.target(m, sys_id, payload, now_us),
            _ => None,
        }
    }

    fn label(&self, en: &str, v: u64) -> String {
        self.dialect
            .label(en, v)
            .unwrap_or_else(|| format!("undefined value {v}"))
    }

    fn track_identity(&mut self, m: &Message, p: &[u8]) -> Option<Event> {
        let uid = uid16(m, p, "track_uid")?;
        let id = Identity {
            stanag_identity: u(m, p, "stanag_identity") as u8,
            target_force: u(m, p, "target_force") as u8,
            target_class: u(m, p, "target_class") as u8,
            environment: u(m, p, "environment") as u8,
            sidc_context: u(m, p, "sidc_context") as u8,
            id_method: u(m, p, "id_method") as u8,
            pid_status: u(m, p, "pid_status") as u8,
            id_confidence: fl(m, p, "id_confidence"),
            external_track_number: text(m, p, "external_track_number"),
            external_track_type: u(m, p, "external_track_type") as u8,
            origin_sysid: u(m, p, "origin_sysid") as u8,
        };
        self.identities.insert(uid, id);
        // re-issue a known track with its new identity
        let k = self.tracks.get(&uid)?.clone();
        let mut e = k.event;
        self.apply_identity(&mut e, &uid, k.class, k.force, k.domain_dim);
        Some(e)
    }

    /// Sets type, callsign prefix and remarks from the stored identity, or
    /// from the message's own class and force when there is none.
    fn apply_identity(
        &self,
        e: &mut Event,
        uid: &[u8; 16],
        class: u8,
        force: u8,
        domain_dim: Option<&'static str>,
    ) {
        let id = self.identities.get(uid);
        let (aff, class, dim) = match id {
            Some(id) => (
                affiliation_from_identity(id.stanag_identity),
                if id.target_class != 0 {
                    id.target_class
                } else {
                    class
                },
                environment_dimension(id.environment).or(domain_dim),
            ),
            None => (affiliation_from_force(force), class, domain_dim),
        };
        let civil = aff == 'n';
        let tail = class_type(class, civil).or(dim).unwrap_or("X");
        e.kind = format!("a-{aff}-{tail}");
        let mut notes = Vec::new();
        if let Some(id) = id {
            match id.sidc_context {
                1 => notes.push("EXERCISE track.".to_string()),
                2 => notes.push("SIMULATION track.".to_string()),
                _ => {}
            }
            notes.push(format!(
                "Identity: {}.",
                self.label("MAVLINK_M_STANAG_IDENTITY", id.stanag_identity as u64)
            ));
            notes.push(format!(
                "ID method: {}; PID: {}.",
                self.label("MAVLINK_M_ID_METHOD", id.id_method as u64),
                self.label("MAVLINK_M_PID_STATUS", id.pid_status as u64)
            ));
            if id.id_confidence.is_finite() {
                notes.push(format!("ID confidence {:.2}.", id.id_confidence));
            }
            if !id.external_track_number.is_empty() {
                notes.push(format!(
                    "External track {} ({}).",
                    id.external_track_number,
                    self.label("MAVLINK_M_TRACK_NUMBER_TYPE", id.external_track_type as u64)
                ));
            }
            if let Some(c) = &mut e.callsign {
                let pre = match id.sidc_context {
                    1 => "EX ",
                    2 => "SIM ",
                    _ => "",
                };
                if !pre.is_empty() && !c.starts_with(pre) {
                    *c = format!("{pre}{c}");
                }
            }
        }
        let base = e.remarks.take().unwrap_or_default();
        let base = base.split(" | ").next().unwrap_or("").to_string();
        e.remarks = Some(if notes.is_empty() {
            base
        } else {
            format!("{base} | {}", notes.join(" "))
        });
    }

    fn producer_link(&self, sys_id: u8) -> Link {
        Link {
            uid: format!("{}{sys_id}", self.opts.platform_uid_prefix),
            kind: "a-f-A".into(),
            relation: "p-p".into(),
        }
    }

    fn base_event(&self, uid: String, time_usec: u64, now_us: u64) -> Event {
        let mut e = Event::new(uid, "a-u-X", now_us, self.opts.stale_secs);
        if time_usec != 0 && time_usec <= now_us {
            e.start_us = time_usec;
        }
        e
    }

    fn position(m: &Message, p: &[u8]) -> Option<Point> {
        let lat = i(m, p, "lat");
        let lon = i(m, p, "lon");
        if lat == 0 && lon == 0 {
            return None;
        }
        let mut pt = Point::new(lat as f64 * 1e-7, lon as f64 * 1e-7, fl(m, p, "alt"));
        if !pt.is_valid() {
            return None;
        }
        if m.field("cov_pos_x").is_some() {
            pt.ce = sigma(fl(m, p, "cov_pos_x").max(fl(m, p, "cov_pos_y")));
            pt.le = sigma(fl(m, p, "cov_pos_z"));
        }
        Some(pt)
    }

    fn handover(&mut self, m: &Message, sys_id: u8, p: &[u8], now_us: u64) -> Option<Event> {
        let point = Self::position(m, p)?;
        let track_uid = uid16(m, p, "track_uid");
        let name = text(m, p, "target_name");
        let to_sys = u(m, p, "target_system");
        let uid = match &track_uid {
            Some(t) => format!("{}track-{}", self.opts.uid_prefix, uuid_string(t)),
            None => format!(
                "{}handover-{sys_id}-{}",
                self.opts.uid_prefix,
                u(m, p, "target_set_id")
            ),
        };
        let mut e = self.base_event(uid, u(m, p, "time_usec"), now_us);
        let valid = u(m, p, "valid_until_usec");
        if valid > now_us {
            e.stale_us = valid;
        }
        e.point = point;
        e.precision_location = Some(("GPS".into(), "MSL".into()));
        e.track = track_from_ned(fl(m, p, "vx"), fl(m, p, "vy"));
        e.callsign = Some(if name.is_empty() {
            match &track_uid {
                Some(t) => format!("TRK {}", &uuid_string(t)[..8]),
                None => format!("HANDOVER {sys_id}"),
            }
        } else {
            name
        });
        e.links.push(self.producer_link(sys_id));
        let class = u(m, p, "target_class") as u8;
        let force = u(m, p, "target_force") as u8;
        let mut r = format!(
            "MAVLink-M TARGET_HANDOVER from system {sys_id} to system {to_sys}. Class: {}. Force: {}.",
            self.label("MAVLINK_M_TARGET_CLASS", class as u64),
            self.label("MAVLINK_M_TARGET_FORCE", force as u64)
        );
        let conf = fl(m, p, "confidence_score");
        if conf.is_finite() {
            r.push_str(&format!(" Confidence {conf:.2}."));
        }
        let media = text(m, p, "match_media_url");
        if !media.is_empty() {
            r.push_str(&format!(
                " Media ({}): {media}.",
                self.label("MAVLINK_M_MATCH_MEDIA_TYPE", u(m, p, "match_media_type"))
            ));
        }
        if let Some(t) = &track_uid {
            r.push_str(&format!(" Track {}.", uuid_string(t)));
        }
        e.remarks = Some(r);
        match track_uid {
            Some(t) => {
                self.apply_identity(&mut e, &t, class, force, None);
                self.tracks.insert(
                    t,
                    Kinematics {
                        event: e.clone(),
                        class,
                        force,
                        domain_dim: None,
                    },
                );
            }
            None => {
                let tail = class_type(class, force == 10).unwrap_or("X");
                e.kind = format!("a-{}-{tail}", affiliation_from_force(force));
            }
        }
        Some(e)
    }

    fn target(&mut self, m: &Message, sys_id: u8, p: &[u8], now_us: u64) -> Option<Event> {
        let point = Self::position(m, p)?;
        let target_id = u(m, p, "target_id");
        let name = text(m, p, "target_name");
        let class = u(m, p, "target_class") as u8;
        let force = u(m, p, "target_force") as u8;
        let domain = u(m, p, "target_domain") as u8;
        let mut e = self.base_event(
            format!("{}target-{sys_id}-{target_id}", self.opts.uid_prefix),
            u(m, p, "time_usec"),
            now_us,
        );
        e.point = point;
        e.precision_location = Some(("GPS".into(), "MSL".into()));
        e.track = track_from_ned(fl(m, p, "vx"), fl(m, p, "vy"));
        e.callsign = Some(if name.is_empty() {
            format!("TGT {sys_id}-{target_id}")
        } else {
            name
        });
        e.links.push(self.producer_link(sys_id));
        let aff = affiliation_from_force(force);
        let tail = class_type(class, aff == 'n')
            .or_else(|| domain_dimension(domain))
            .unwrap_or("X");
        e.kind = format!("a-{aff}-{tail}");
        let mut r = format!(
            "MAVLink-M TARGET {target_id} from system {sys_id}. Class: {}. Domain: {}. Force: {}.",
            self.label("MAVLINK_M_TARGET_CLASS", class as u64),
            self.label("MAVLINK_M_TARGET_DOMAIN", domain as u64),
            self.label("MAVLINK_M_TARGET_FORCE", force as u64)
        );
        let conf = u(m, p, "confidence");
        if conf > 0 && conf <= 10_000 {
            r.push_str(&format!(" Confidence {:.2}.", conf as f64 / 10_000.0));
        }
        r.push_str(&format!(
            " Sensor: {}.",
            self.label("MAVLINK_M_TARGET_SENSOR_TYPE", u(m, p, "sensor_type"))
        ));
        // descriptive data-quality flags only
        let flags = u(m, p, "flags");
        let shown: Vec<String> = [2u64, 4, 8, 16, 32, 64, 256, 512]
            .iter()
            .filter(|&&b| flags & b != 0)
            .filter_map(|&b| self.dialect.label("MAVLINK_M_TARGET_FLAGS", b))
            .collect();
        if !shown.is_empty() {
            r.push_str(&format!(" Flags: {}.", shown.join(", ")));
        }
        e.remarks = Some(r);
        Some(e)
    }

    fn participant(&mut self, m: &Message, sys_id: u8, p: &[u8], now_us: u64) -> Option<Event> {
        let point = Self::position(m, p)?;
        let origin = u(m, p, "origin_sysid") as u8;
        let origin = if origin == 0 { sys_id } else { origin };
        let mut e = self.base_event(
            format!("{}ppli-{origin}", self.opts.uid_prefix),
            u(m, p, "time_usec"),
            now_us,
        );
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
                " External track {etn} ({}).",
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
