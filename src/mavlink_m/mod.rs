//! MAVLink-M (the Dronecode military MAVLink dialect, `military.xml`)
//! friendly position reports to CoT, and a conformance checker for
//! MAVLink-M framing.
//!
//! Scope: interoperability and telemetry only. [`Gateway`] turns
//! `PARTICIPANT_POSITION` (friendly self-reports, PPLI-style) into CoT
//! markers, so ATAK, WinTAK and TAK Server show friendly positions next to
//! the vehicles that [`crate::mavlink`] maps from `HEARTBEAT` and
//! `GLOBAL_POSITION_INT`. Nothing in the targeting chain is handled: every
//! target, track, handover, fires, munition, arming, weapon and engagement
//! message is refused at compile time (the translated list cannot contain
//! one) and at run time (the gateway drops them, and [`Message`] will not
//! decode or encode them). See [`scope`].
//!
//! [`validate`] checks frames against the dialect definition: ids,
//! lengths and `CRC_EXTRA` checksums for every message, plus enum values
//! (including the extended-Hamming protected enums), bitmasks and stated
//! ranges for the in-scope messages only. Out-of-scope messages are never
//! decoded.
//!
//! The dialect is the embedded copy of `military.xml` from
//! [Dronecode/mavlink-military](https://github.com/Dronecode/mavlink-military)
//! (MIT), see [`military`]; [`Dialect::from_file`] loads another version.
//!
//! ```
//! use uas_cot::mavlink_m::{self, Gateway, Value};
//!
//! let d = mavlink_m::military();
//! let m = d.message_by_name("PARTICIPANT_POSITION").unwrap();
//! let payload = m.encode(&[
//!     ("lat", Value::Int(473_977_418)),
//!     ("lon", Value::Int(85_455_939)),
//!     ("alt", Value::Float(488.0)),
//!     ("callsign", Value::Text("RAVEN 1".into())),
//!     ("stanag_identity", Value::Uint(3)), // FRIEND
//!     ("ppli_type", Value::Uint(1)),       // AIR
//! ]).unwrap();
//! let report = mavlink_m::validate::check_payload(d, m.id, &payload, true);
//! assert!(report.is_ok(), "{report}");
//!
//! let mut gw = Gateway::default();
//! let e = gw.ingest(7, m.id, &payload, 1_700_000_000_000_000).unwrap();
//! assert_eq!(e.kind, "a-f-A");
//! assert_eq!(e.callsign.as_deref(), Some("RAVEN 1"));
//! ```

mod cot;
mod dialect;
pub mod scope;
pub mod validate;

pub use cot::{affiliation_from_identity, ppli_dimension, Gateway, Options};
pub use dialect::{
    crc_accumulate, layout, Dialect, Entry, Enum, Error, Field, FieldType, Message, Value,
};

use std::sync::OnceLock;

/// `PARTICIPANT_POSITION` message id, the one MAVLink-M message the
/// gateway translates.
pub const PARTICIPANT_POSITION: u32 = 53003;

/// The embedded `military.xml` (Dronecode/mavlink-military commit
/// `6375c3447c08`, 2026-10-28).
pub const MILITARY_XML: &str = include_str!("../../dialects/military.xml");

/// Commit of Dronecode/mavlink-military the embedded definition comes from.
pub const MILITARY_XML_COMMIT: &str = "6375c3447c08ee41efc4803d4533af6d64478c8c";

/// The embedded MAVLink-M definition, parsed once. It holds the dialect's
/// own messages and enums; `common.xml`, which it includes, is not
/// embedded, so enums it only extends (for example `MAV_CMD`) hold just the
/// MAVLink-M entries.
pub fn military() -> &'static Dialect {
    static D: OnceLock<Dialect> = OnceLock::new();
    D.get_or_init(|| {
        Dialect::parse("military", MILITARY_XML).expect("embedded military.xml parses")
    })
}

/// `CRC_EXTRA` of a MAVLink-M message id, for frame parsers that take a
/// lookup function (for example `mavlink_tlog::CrcPolicy::Custom`).
pub fn crc_extra(msg_id: u32) -> Option<u8> {
    military().crc_extra(msg_id)
}
