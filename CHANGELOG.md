# Changelog

## 0.2.0 (2026-11-17)

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
