//! What this crate will and will not handle from the MAVLink-M dialect.
//!
//! uas-cot is interoperability and telemetry only. It puts positions and
//! vehicle state on a TAK map. It does not relay anything in the targeting
//! or engagement chain: target, track, handover, fires, munition, arming,
//! weapon station, close air support, terminal control or engagement
//! messages, and the acknowledgments that close those loops.
//!
//! The block works at three levels:
//!
//! - Compile time: [`TRANSLATED`] is the complete list of MAVLink-M messages
//!   the [`super::Gateway`] turns into CoT. A `const` assertion in this
//!   module stops the crate from building if any entry in it is also in
//!   [`BLOCKED`] or has a blocked word in its name.
//! - Run time: [`super::Gateway::ingest`] drops every message for which
//!   [`is_blocked`] is true before it looks at the payload, and counts it.
//!   [`super::Message::get`], [`super::Message::decode`] and
//!   [`super::Message::encode`] refuse blocked messages, so nothing in the
//!   crate decodes or builds their field contents.
//! - Validation: [`super::validate`] checks framing, checksum and length of
//!   blocked messages, but never decodes or prints their fields.
//!
//! [`is_blocked`] also applies a name rule to the MAVLink-M id range
//! (53000 to 53999), so a newer `military.xml` loaded with
//! [`super::Dialect::from_file`] cannot slip a renamed or new target or
//! fires message past the list.

/// MAVLink-M messages that are out of scope: target, track, handover,
/// fires, munition, arming, weapon, engagement and their acknowledgment.
/// Ids and names from `military.xml`.
pub const BLOCKED: &[(u32, &str)] = &[
    (53000, "TRACK_IDENTITY"),
    (53001, "TARGET_CUE"),
    (53002, "TARGET_HANDOVER"),
    (53004, "MAVLINK_M_ACK"),
    (53010, "TARGET"),
    (53011, "TARGET_SET_COORD"),
    (53012, "TARGET_BOX_COORD"),
    (53013, "TARGET_AUTHORIZATION"),
    (53020, "FIRES"),
    (53021, "SPLASH_CORRECTION"),
    (53022, "BATTLE_DAMAGE_ASSESSMENT"),
    (53023, "ENGAGEMENT_DIRECTIVE"),
    (53024, "CALL_FOR_FIRE"),
    (53030, "ESAD_STATE"),
    (53031, "ESAD_ARMING"),
    (53032, "ESAD_CONFIG"),
    (53033, "STORE_STATUS"),
    (53034, "STORE_SENSOR_STATUS"),
    (53035, "STORE_MUNITION"),
    (53036, "LOITER_MUNITION_CONTROL"),
    (53040, "RWS_POSE"),
    (53041, "RWS_STATE"),
    (53050, "SENSOR_TASKING"),
    (53060, "CAS_9LINE"),
    (53061, "TERMINAL_CONTROL"),
];

/// Words that mark a MAVLink-M message (id 53000 to 53999) as out of scope
/// whatever its id, so a newer dialect is blocked by name too.
pub const BLOCKED_WORDS: &[&str] = &[
    "TARGET",
    "TRACK",
    "FIRE",
    "MUNITION",
    "ENGAGE",
    "HANDOVER",
    "ESAD",
    "ARMING",
    "WEAPON",
    "STRIKE",
    "SPLASH",
    "BATTLE_DAMAGE",
    "CAS_",
    "9LINE",
    "TERMINAL_CONTROL",
    "RWS_",
    "STORE_",
    "TASKING",
    "MAVLINK_M_ACK",
];

/// Every MAVLink-M message the gateway translates to CoT: friendly
/// position reports (PPLI) only. Standard vehicle telemetry
/// (`HEARTBEAT`, `GLOBAL_POSITION_INT`) is handled by [`crate::mavlink`].
pub const TRANSLATED: &[(u32, &str)] = &[(super::PARTICIPANT_POSITION, "PARTICIPANT_POSITION")];

/// The MAVLink-M message id range.
pub const MAVLINK_M_IDS: std::ops::RangeInclusive<u32> = 53_000..=53_999;

const fn contains(hay: &[u8], needle: &[u8]) -> bool {
    if needle.len() > hay.len() {
        return false;
    }
    let mut i = 0;
    while i + needle.len() <= hay.len() {
        let mut j = 0;
        while j < needle.len() && hay[i + j] == needle[j] {
            j += 1;
        }
        if j == needle.len() {
            return true;
        }
        i += 1;
    }
    false
}

/// True if `id` is in [`BLOCKED`].
pub const fn is_blocked_id(id: u32) -> bool {
    let mut i = 0;
    while i < BLOCKED.len() {
        if BLOCKED[i].0 == id {
            return true;
        }
        i += 1;
    }
    false
}

/// True if `name` is in [`BLOCKED`] or contains one of [`BLOCKED_WORDS`].
pub const fn is_blocked_name(name: &str) -> bool {
    let n = name.as_bytes();
    let mut i = 0;
    while i < BLOCKED_WORDS.len() {
        if contains(n, BLOCKED_WORDS[i].as_bytes()) {
            return true;
        }
        i += 1;
    }
    let mut i = 0;
    while i < BLOCKED.len() {
        let b = BLOCKED[i].1.as_bytes();
        if b.len() == n.len() && contains(n, b) {
            return true;
        }
        i += 1;
    }
    false
}

/// True if message `id` called `name` is out of scope: listed in
/// [`BLOCKED`], or a MAVLink-M id whose name has a blocked word.
pub fn is_blocked(id: u32, name: &str) -> bool {
    is_blocked_id(id) || (MAVLINK_M_IDS.contains(&id) && is_blocked_name(name))
}

/// True if the gateway translates message `id`.
pub fn is_translated(id: u32) -> bool {
    TRANSLATED.iter().any(|&(t, _)| t == id) && !is_blocked_id(id)
}

// Compile-time block: nothing the gateway translates may be blocked.
const _: () = {
    let mut i = 0;
    while i < TRANSLATED.len() {
        assert!(
            !is_blocked_id(TRANSLATED[i].0),
            "a translated MAVLink-M message is on the blocked list"
        );
        assert!(
            !is_blocked_name(TRANSLATED[i].1),
            "a translated MAVLink-M message has a blocked word in its name"
        );
        i += 1;
    }
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn name_rule() {
        assert!(is_blocked_name("CALL_FOR_FIRE"));
        assert!(is_blocked_name("TARGET_SET_COORD"));
        assert!(is_blocked_name("LOITER_MUNITION_CONTROL"));
        assert!(is_blocked_name("ESAD_ARMING"));
        assert!(!is_blocked_name("PARTICIPANT_POSITION"));
        assert!(!is_blocked_name("PAYLOAD_STATE"));
        // the name rule only applies inside the MAVLink-M range: common.xml
        // position setpoints such as POSITION_TARGET_GLOBAL_INT are not
        // MAVLink-M messages
        assert!(!is_blocked(87, "POSITION_TARGET_GLOBAL_INT"));
        assert!(is_blocked(53_500, "TARGET_NEW"));
        assert!(is_blocked(53_020, "RENAMED"));
    }
}
