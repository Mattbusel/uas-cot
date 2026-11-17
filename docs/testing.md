# How it is tested

## Inputs (`tests/data`)

| File | Source |
|---|---|
| `day_flight.klv`, `night_flight_ir.klv` | KLV streams of the public STANAG 4609 samples at https://samples.ffmpeg.org/MPEG2/mpegts-klv/ |
| `odid_wifi_bcn_sample.pcap`, `odid_wifi_sample.pcap` | Remote ID Wi-Fi Beacon and NAN captures from [opendroneid/wireshark-dissector](https://github.com/opendroneid/wireshark-dissector) (Apache-2.0, `LICENSE-APACHE-opendroneid`) |
| synthetic `.tlog` | built by the test with `mavlink-tlog`: HEARTBEAT + GLOBAL_POSITION_INT, MAVLink 2 with trimmed payloads |

## Checks

- `tests/end_to_end.rs` runs the `uas2cot` binary on each input and
  compares the output with `tests/data/expected/*.cot` (164 events). It
  also checks specific values (the Remote ID sample aircraft at 45.5470818,
  -122.9668346, 237 m, course 280, 20.5 m/s; the KLV frame center; course
  from velocity when MAVLink heading is unknown) and sends over UDP.
- Every expected event is validated against the MITRE CoT public release
  schemas (event, contact, track, sensor, link, remarks; from
  `deptofdefense/AndroidTacticalAssaultKit-CIV/takcot/mitre`):
  `python tools/validate_cot.py tests/schema/cot-all.xsd tests/data/expected/*.cot`.
- Every expected event is converted to TAK Protocol v1 (mesh and stream)
  with [takproto](https://pypi.org/project/takproto/) and read back; uid,
  type, position, callsign, course and speed must survive:
  `python tools/takproto_check.py tests/data/expected/*.cot`.
- Unit tests: XML escaping, time formatting, angle normalisation, MAVLink
  payload trimming, UDP and TCP delivery.

Regenerate the expected files after an intended change:

```
UAS_COT_REGENERATE=1 cargo test
```
