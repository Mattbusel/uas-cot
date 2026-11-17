//! MAVLink-M (the Dronecode military MAVLink dialect, `military.xml`) to
//! CoT, and a conformance checker for MAVLink-M traffic.
//!
//! MAVLink-M carries shared tracks between platforms: a persistent track
//! identity (`TRACK_IDENTITY`), custody transfer of a tracked object
//! between systems (`TARGET_HANDOVER`), friendly self-reports
//! (`PARTICIPANT_POSITION`, PPLI-style) and individual object reports
//! (`TARGET`). [`Gateway`] turns those into CoT events, so ATAK, WinTAK and
//! TAK Server show what a MAVLink-M platform sees, next to the vehicles
//! that [`crate::mavlink`] already maps.
//!
//! This is interoperability and telemetry only: field translation of the
//! public dialect onto a map picture. Engagement fields (authorization
//! tokens, laser codes, target packages, CEP, DMPI, restricted-target
//! flags) are never read into an event.
//!
//! [`validate`] checks frames and payloads against the dialect definition:
//! ids, lengths, `CRC_EXTRA` checksums, enum values (including the
//! extended-Hamming protected enums), bitmasks and the ranges the
//! definition states.
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
pub mod validate;

pub use cot::{
    affiliation_from_force, affiliation_from_identity, class_type, domain_dimension,
    environment_dimension, ppli_dimension, uuid_string, Gateway, Identity, Options,
};
pub use dialect::{
    crc_accumulate, layout, Dialect, Entry, Enum, Error, Field, FieldType, Message, Value,
};

use std::sync::OnceLock;

/// `TRACK_IDENTITY` message id.
pub const TRACK_IDENTITY: u32 = 53000;
/// `TARGET_CUE` message id.
pub const TARGET_CUE: u32 = 53001;
/// `TARGET_HANDOVER` message id.
pub const TARGET_HANDOVER: u32 = 53002;
/// `PARTICIPANT_POSITION` message id.
pub const PARTICIPANT_POSITION: u32 = 53003;
/// `TARGET` message id.
pub const TARGET: u32 = 53010;

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
