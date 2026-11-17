# Vendored dialect

`military.xml` is the MAVLink-M dialect from
https://github.com/Dronecode/mavlink-military at commit
`6375c3447c08ee41efc4803d4533af6d64478c8c` (2026-10-28), unchanged.
Copyright (c) 2026 Dronecode Project, Inc, MIT License (`LICENSE-MIT-dronecode`).

The `mavlink-m` feature embeds it so the validator and the CoT gateway work
without network access. `Dialect::from_file` loads a newer copy (and its
`<include>common.xml</include>`) when you have one.

The full definition is kept because checksums (`CRC_EXTRA`) and lengths
need every message. uas-cot only translates `PARTICIPANT_POSITION`; the
target, track, fires, munition and engagement messages are blocked (see
`src/mavlink_m/scope.rs` and `docs/mavlink-m.md`).
