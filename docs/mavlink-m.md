# MAVLink-M friendly positions to CoT, and framing checks

[MAVLink-M](https://mavlink-m.dronecode.org/) is the military dialect of
MAVLink 2 that Dronecode publishes as `military.xml` in
[Dronecode/mavlink-military](https://github.com/Dronecode/mavlink-military)
(MIT). uas-cot's `mavlink-m` feature puts MAVLink-M friendly position
reports on a TAK map, next to the vehicle markers uas-cot already makes
from `HEARTBEAT` and `GLOBAL_POSITION_INT`, and ships a checker that tests
MAVLink-M framing against the definition.

## Scope

Telemetry and position interoperability only. Nothing in the targeting or
engagement chain.

| | Messages | What uas-cot does |
|---|---|---|
| Translated to CoT | `PARTICIPANT_POSITION` (53003); vehicle `HEARTBEAT` and `GLOBAL_POSITION_INT` (common MAVLink) | markers on the map |
| Blocked | `TRACK_IDENTITY`, `TARGET_CUE`, `TARGET_HANDOVER`, `TARGET`, `TARGET_SET_COORD`, `TARGET_BOX_COORD`, `TARGET_AUTHORIZATION`, `FIRES`, `SPLASH_CORRECTION`, `BATTLE_DAMAGE_ASSESSMENT`, `ENGAGEMENT_DIRECTIVE`, `CALL_FOR_FIRE`, `ESAD_STATE`, `ESAD_ARMING`, `ESAD_CONFIG`, `STORE_STATUS`, `STORE_SENSOR_STATUS`, `STORE_MUNITION`, `LOITER_MUNITION_CONTROL`, `RWS_POSE`, `RWS_STATE`, `SENSOR_TASKING`, `CAS_9LINE`, `TERMINAL_CONTROL`, `MAVLINK_M_ACK`; the generic payload messages `PAYLOAD_INFO`, `PAYLOAD_STATE`, `PAYLOAD_RESOURCE`, `PAYLOAD_INHIBIT`, `PAYLOAD_MEASUREMENT`, `PAYLOAD_OPERATION_REQUEST`, `PAYLOAD_OPERATION_STATUS`, `PAYLOAD_CATALOGUE_STATUS`, `PAYLOAD_OPERATION_CANCEL` (53100 to 53108), which can report payload arming and activation state | never translated, never decoded; the checker sees framing, length and checksum only |

How the block is enforced (`src/mavlink_m/scope.rs`):

- **Compile time.** `scope::TRANSLATED` is the complete list of MAVLink-M
  messages the gateway turns into CoT. A `const` assertion fails the build
  if any entry is on `scope::BLOCKED` or has a blocked word in its name.
- **Run time.** `Gateway::ingest` drops every blocked message before it
  reads the payload, returns no event, and counts it (`Gateway::blocked`).
  `Message::get`, `Message::decode` and `Message::encode` refuse blocked
  messages, so the crate never decodes or builds their fields.
- **Newer dialects.** Besides the list of 34, any message in the MAVLink-M
  id range (53000 to 53999) whose name contains `TARGET`, `TRACK`, `FIRE`,
  `MUNITION`, `ENGAGE`, `HANDOVER`, `ESAD`, `ARMING`, `WEAPON`, `STRIKE`,
  `PAYLOAD_` and the other words in `scope::BLOCKED_WORDS` is blocked, so a
  `military.xml` loaded with `--dialect` cannot slip a new or renamed one
  past the list.
- **Tests.** `tests/mavlink_m.rs` fails if any blocked message produces
  CoT (zero-filled, all-ones, or a plausible payload that would pass
  validation), is decoded, or has a field printed by the checker, and if
  any message in `military.xml` is not classified as blocked or in scope.

![uas2cot output for the simulated scenario](mavlink-m-demo.gif)

*The GIF is drawn from `uas2cot` output (every frame is a CoT event the
gateway produced) for the scripted, simulated scenario in
`examples/mavlink_m_sim.rs`. It is not a recording of real aircraft or of a
TAK client.*

## Try it

```
cargo install uas-cot --features mavlink-m

# translate live MAVLink + MAVLink-M from UDP to the TAK SA multicast group
uas2cot mavlink-udp 0.0.0.0:14550 --udp 239.2.3.1:6969

# check MAVLink-M traffic against military.xml (exit code 1 on any error)
mavlinkm-check udp 0.0.0.0:14550 --count 500
mavlinkm-check tlog flight.tlog
mavlinkm-check raw capture.bin          # back-to-back frames, for example a serial capture
mavlinkm-check tlog flight.tlog --dialect path/to/mavlink-military/military.xml
```

Without hardware, from a clone of this repository:

```
cargo run --features mavlink-m --example mavlink_m_sim -- --tlog demo.tlog --secs 90 --fault
cargo run --features mavlink-m --bin uas2cot -- tlog demo.tlog > demo.cot
cargo run --features mavlink-m --bin mavlinkm-check -- tlog demo.tlog
python tools/validate_cot.py tests/schema/cot-all.xsd demo.cot
python tools/cot_gif.py demo.cot demo.gif
```

Live, in two terminals:

```
cargo run --features mavlink-m --bin uas2cot -- mavlink-udp 127.0.0.1:14560
cargo run --features mavlink-m --example mavlink_m_sim -- --udp 127.0.0.1:14560 --secs 60
```

To use a PX4 or ArduPilot SITL as the vehicle side, point the gateway at
the SITL's MAVLink UDP output (PX4 SITL sends to port 14550 by default).
The SITL supplies `HEARTBEAT` and `GLOBAL_POSITION_INT`; MAVLink-M
position reports come from whatever node on the network speaks the
dialect (or from the simulator above on the same port).

What the scenario contains, all synthetic, around the PX4 SITL default
home (47.397742, 8.545594):

| Source | Messages | CoT |
|---|---|---|
| System 1, quadrotor | `HEARTBEAT`, `GLOBAL_POSITION_INT` 10 Hz | `mav-1`, `a-f-A-M-F-Q` |
| System 2, fixed wing | `HEARTBEAT`, `GLOBAL_POSITION_INT` 10 Hz | `mav-2`, `a-f-A-M-F-Q` |
| Ground team "ANVIL 3", relayed by system 1 | `PARTICIPANT_POSITION` 1 Hz | `mavlink-m-ppli-3`, `a-f-G` |
| Survey boat "HERON 4", relayed by system 2 | `PARTICIPANT_POSITION` 1 Hz | `mavlink-m-ppli-4`, `a-f-S` |
| `--fault` | one `PARTICIPANT_POSITION` with `ppli_type = 9` (undefined) | dropped by the gateway, reported by the checker |

Output of the steps above on this machine (Windows 11):

```
mavlink_m_sim: wrote 2161 records to demo.tlog
uas2cot: 1980 events
PARTICIPANT_POSITION (53003) from 1/1
  error [ppli_type]: 9 is not a defined MAVLINK_M_PPLI_TYPE value
mavlinkm-check: 181 MAVLink-M messages checked against military (embedded, commit 6375c3447c08), 1 with errors
  PARTICIPANT_POSITION: 181 checked, 1 with errors
1980 events, 0 invalid                     (MITRE CoT schemas)
1980 events converted to TAK Protocol v1 (mesh and stream) and checked
```

## Mapping

| MAVLink-M | CoT |
|---|---|
| `PARTICIPANT_POSITION` (53003) | `uid` `mavlink-m-ppli-<origin_sysid>` (the sender when 0); type `a-<identity>-<ppli_type dimension>`; `point` from `lat`, `lon` (degE7), `alt` (MSL, `altsrc="MSL"`, 0 when NaN); `track` from `vx`, `vy`, with `course` when given; `callsign`; remarks: type, identity, the participant's network number |

Affiliation: `stanag_identity` maps pending, unknown, assumed friend,
friend, neutral, suspect, hostile to CoT `p u a f n s h`. Dimension:
`ppli_type` air, land, surface, subsurface to `A G S U`, otherwise `X`.

## Conformance checks

`mavlinkm-check` and `uas_cot::mavlink_m::validate` read every rule from
the definition, so a newer `military.xml` passed with `--dialect` is
checked by its own rules.

All MAVLink-M messages, blocked ones included:

| Check | From the XML |
|---|---|
| message id is defined | `<message id>` |
| length: MAVLink 1 exact; MAVLink 2 not longer than the message, trailing zero bytes trimmed (warning if not) | field types, `<extensions/>` |
| checksum with the message's `CRC_EXTRA` (a mismatch means the sender built from a different definition, or corruption) | field names, types, order |

In-scope messages only (in the embedded dialect, `PARTICIPANT_POSITION`):

| Check | From the XML |
|---|---|
| enum fields hold a defined value | `enum=`, `<entry>` |
| protected enums (values pairwise 4 or more bits apart): an undefined value is reported as corruption, with the nearest codeword and its distance; the dialect says receivers reject it rather than correct it | entry values |
| bitmask fields set no undefined bits | `bitmask="true"` |
| `degE7` latitude and longitude in range | `units="degE7"` |
| ranges stated in descriptions: `[0.0-1.0]`, `[0-100]` with `255 = N/A`, `0 to 10000`; "Never 0" | `<description>` |
| `char[N]` is valid UTF-8 | types |

A blocked message is reported as
`NAME (id) from sys/comp [out of scope: framing, length and checksum only]: ok`
(or with its framing errors). Its fields are never decoded, checked or
printed.

The gateway runs the same checks and drops a message that fails
(`Options::reject_invalid`, on by default).

How the layout is verified: `tests/data/mavlink_m_reference.txt` holds
`CRC_EXTRA`, base length and full length of all 35 MAVLink-M messages as
pymavlink 2.4.50 computes them; `layout_matches_pymavlink` compares the
crate's own computation with every one. Checksums need this for every
message, which is why the frame checks cover blocked messages too.

## Version

The embedded `military.xml` is Dronecode/mavlink-military commit
`6375c3447c08` (2026-10-28), unchanged, in `dialects/`. MAVLink-M is still
moving; pass `--dialect` (or `Dialect::from_file`) to use a newer copy.
The scope block applies to it as well.
