# Changelog

## 0.6.0 (2026-11-22)

New features; no existing function, type or field changes. The minor bump
is for the `uas2cot` output change below.

- `rid::AffiliationRules` and `rid::tracked_with`: the aircraft's CoT
  affiliation from its remoteid trust verdict and a list of your own Basic
  IDs or Operator IDs. Verdict rules come first, so a spoofed or
  suspicious transmitter that copies one of your serials is never drawn
  friendly. Defaults: spoofed to suspect (`s`), everything else unchanged.
- `rid::operator_or_takeoff`: the System message ground position labelled
  by its location type. A takeoff point is its own entity
  (`rid-takeoff-<id>`, `Takeoff point of <id>`), a live or fixed operator
  location keeps `rid-op-<id>`. `rid::operator` is unchanged.
- Feature `tls`: `net::TlsSender` and `net::TlsConfig`, TLS streaming to a
  TAK Server input (8089) with a client certificate, on rustls with the
  ring provider (no OpenSSL). PEM CA, certificate and key;
  `TlsConfig::from_pem_any_name` accepts a server certificate that chains
  to your CA but does not name the dialled host; `TlsConfig::from_rustls`
  takes your own `rustls::ClientConfig`. `uas_cot::rustls` re-exports it.
  The handshake completes in `connect`, so a bad certificate fails there.
- `uas2cot`: `--tls addr --ca --cert --key [--tls-any-name]` (feature
  `tls`), `--friendly <id>` (repeatable), `--spoofed-as <c>`,
  `--suspicious-as <c>`. `uas2cot pcap` now uses `rid::tracked_with`: a
  takeoff point is sent as `rid-takeoff-<id>` instead of `rid-op-<id>`
  (both Open Drone ID sample captures change only there), and a spoofed
  verdict draws the aircraft as suspect.

## 0.5.0 (2026-11-21)

- Depends on remoteid 0.3 (was 0.2), so `uas_cot::remoteid` now carries
  `remoteid::capture`: pcap, pcapng and RIDF captures straight into a
  `Tracker` and on to `rid::tracked`. Breaking only because the re-exported
  remoteid moved a minor version.

## 0.4.0 (2026-11-21)

- `Sensor::north`: the ATAK `<sensor north>` image north bearing, written
  wrapped to 0..360. Adding the field is a breaking change for struct
  literals without `..Default::default()`.

## 0.3.2 (2026-11-20)

Documentation and metadata only; no code change.

- README opens with what reaches the TAK map: open Remote ID receivers
  flagged 0 of 10 spoofed transmitters, and `uas2cot pcap` puts the
  remoteid trust verdict on every Remote ID aircraft marker it sends.
- crates.io description and keywords say the same.

## 0.3.1 (2026-11-18)

- `rid::tracked`: the aircraft and operator events for a
  `remoteid::TrackedUas`, with the transmitter's trust verdict and reasons
  appended to the aircraft remarks (`Trust: spoofed (known_spoofer,
  teleport; tool: ...).`). `rid::trust_remarks` formats that sentence
  from any `remoteid::Assessment`. Until now a TAK user saw a spoofed
  Remote ID broadcast as an ordinary aircraft marker.
- `uas2cot pcap` uses it, so every Remote ID aircraft marker it sends
  carries the verdict. The expected output for the two Open Drone ID
  sample captures changes only by ` Trust: plausible.` in each aircraft's
  remarks; a new test runs a capture of the jjshoots/RemoteIDSpoofer
  ESP8266 sketch and checks the marker turns `suspicious`.
- The `rid_to_cot` example and `docs/integration.md` use `rid::tracked`.
- No change to existing functions, types or the other event fields.

## 0.3.0 (2026-11-17)

Integration release. Breaking only through the `remoteid` upgrade.

- Builds against `remoteid` 0.2 (was 0.1). `rid::aircraft` and
  `rid::operator` take `remoteid::UasData`, so a project already on
  `remoteid` 0.2 (trust verdicts, DRIP verification) got a type mismatch
  from uas-cot 0.2 and a second copy of `remoteid` in its tree. It now
  shares one copy.
- Re-exports `uas_cot::remoteid` and `uas_cot::misb`, the exact versions
  this build uses.
- `misb` requirement raised to 0.1.1 (version byte fix in key matching).
- New `examples/rid_to_cot.rs` and `docs/integration.md`: features,
  the API to depend on, MSRV and versioning.

## 0.2.2 (2026-11-17)

Scope correction. The nine generic payload messages of `military.xml` can
report and request payload arming and activation state, so they are now
blocked like the targeting messages.

- `scope::BLOCKED` adds `PAYLOAD_INFO`, `PAYLOAD_STATE`,
  `PAYLOAD_RESOURCE`, `PAYLOAD_INHIBIT`, `PAYLOAD_MEASUREMENT`,
  `PAYLOAD_OPERATION_REQUEST`, `PAYLOAD_OPERATION_STATUS`,
  `PAYLOAD_CATALOGUE_STATUS` and `PAYLOAD_OPERATION_CANCEL` (53100 to
  53108), 34 blocked messages in all. `scope::BLOCKED_WORDS` adds
  `PAYLOAD_`, so a newer dialect's payload messages are blocked by name.
- `Message::get`, `decode` and `encode` refuse them; the gateway drops and
  counts them.
- `validate` and `mavlinkm-check` give them the frame checks only (id,
  length, checksum) with the `[out of scope: framing, length and checksum
  only]` label; their fields are no longer decoded, checked or printed.
- The only in-scope MAVLink-M message in the embedded dialect is now
  `PARTICIPANT_POSITION`.
- Tests: `every_message_is_classified`, `blocked_messages_never_produce_cot`
  and the other block tests cover the payload messages; the protected-enum
  check is tested with a small dialect of its own.

## 0.2.1 (2026-11-17)

Scope correction. uas-cot is telemetry and position interoperability only;
0.2.0 translated MAVLink-M targeting messages and is yanked.

- The gateway no longer translates `TRACK_IDENTITY`, `TARGET_HANDOVER` or
  `TARGET`. It translates `PARTICIPANT_POSITION` (friendly positions) only;
  vehicle telemetry (`HEARTBEAT`, `GLOBAL_POSITION_INT`) is unchanged.
- New `mavlink_m::scope`: every target, track, handover, fires, munition,
  arming, weapon station, CAS, terminal control and engagement message of
  `military.xml` (25 messages, including `CALL_FOR_FIRE`, `FIRES`,
  `ESAD_ARMING`, `LOITER_MUNITION_CONTROL`, `TARGET*`, `TRACK*`) is
  blocked, plus a name rule over the MAVLink-M id range for newer
  dialects.
  - Compile time: a `const` assertion fails the build if the translated
    list holds a blocked message.
  - Run time: `Gateway::ingest` drops blocked messages before reading them
    and counts them (`Gateway::blocked`); `Message::get`, `decode` and
    `encode` refuse them.
- `validate` and `mavlinkm-check`: blocked messages get the frame checks
  only (id, length, checksum) and are reported as
  `[out of scope: framing, length and checksum only]`; their fields are
  never decoded, range-checked or printed (`Report::framing_only`).
- Removed: `Identity`, `uuid_string`, `class_type`, `affiliation_from_force`,
  `domain_dimension`, `environment_dimension`, `Options::platform_uid_prefix`
  and the `TRACK_IDENTITY`, `TARGET_CUE`, `TARGET_HANDOVER` and `TARGET`
  id constants.
- The simulated demo (`examples/mavlink_m_sim.rs`) now sends vehicle
  telemetry and friendly position reports only; the GIF is redrawn.
- Tests that fail if any blocked message produces CoT, is decoded, or has
  a field printed by the checker.

## 0.2.0 (2026-11-17, yanked: out of scope, see 0.2.1)

- New feature `mavlink-m`: the `mavlink_m` module and the `mavlinkm-check`
  binary, built on the embedded MAVLink-M dialect (`military.xml` from
  Dronecode/mavlink-military commit 6375c3447c08, MIT).
  - `Gateway`: `TRACK_IDENTITY`, `TARGET_HANDOVER`, `PARTICIPANT_POSITION`
    and `TARGET` to CoT. Identity joins handovers by `track_uid`;
    affiliation from the standard identity or force; 2525B function from
    the target class; exercise and simulation tracks are marked. Engagement
    fields (authorization tokens, laser codes, target packages, CEP, DMPI,
    restricted-target flags) are never translated.
  - `Dialect`: MAVLink XML loader (with includes), MAVLink 2 field order,
    offsets and `CRC_EXTRA` (matches pymavlink for all 35 MAVLink-M
    messages), payload encode and decode.
  - `validate`: frame and payload checks against the definition: ids,
    lengths, checksums, enum values (with the extended-Hamming protected
    enums), bitmasks, degE7 ranges and the ranges the descriptions state.
  - `uas2cot tlog` and `uas2cot mavlink-udp` translate MAVLink-M messages
    when built with the feature.
  - `examples/mavlink_m_sim.rs`: a scripted, simulated MAVLink + MAVLink-M
    emitter (`--tlog` or `--udp`, `--fault`); `tools/cot_gif.py` draws
    uas2cot output as a GIF.
- `uas2cot tlog` now checks frame checksums (built-in table, plus MAVLink-M
  with the feature) and skips frames that fail.

## 0.1.0 (2026-11-17)

- CoT 2.0 event model and XML writer (`Event`, `Point`, `Track`, `Sensor`, `Link`).
- `rid`: Remote ID aircraft and operator events from `remoteid::UasData`.
- `fmv`: MISB ST 0601 platform (with `<sensor>` field of view and video URL)
  and sensor point of interest events from `misb::st0601::Packet`.
- `mavlink`: `GLOBAL_POSITION_INT` and `HEARTBEAT` payload decoding and
  vehicle events, no MAVLink dependency.
- `net`: UDP (SA multicast 239.2.3.1:6969) and TCP senders.
- `uas2cot` binary: `klv`, `tlog`, `pcap` and `mavlink-udp` inputs; stdout,
  `--udp` and `--tcp` outputs; `--realtime` replay.
- Output validated against the MITRE CoT schemas and converted to TAK
  Protocol v1 with takproto in CI.
