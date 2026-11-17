# Changelog

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
