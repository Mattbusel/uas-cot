# Using uas-cot inside another project

uas-cot turns drone telemetry you already decode into Cursor on Target
events. It does not own your sockets, threads or radios: you call a function
with a decoded record and get an `Event` back, then send it however your
system sends CoT.

## Add it

For a library or service, turn off the command line tools:

```toml
[dependencies]
uas-cot = { version = "0.3", default-features = false, features = ["remoteid", "misb"] }
```

| Feature | Default | Adds | Extra dependencies |
|---|---|---|---|
| `remoteid` | yes | `rid`: Remote ID records to CoT | `remoteid` (no dependencies of its own) |
| `misb` | yes | `fmv`: MISB ST 0601 KLV to CoT | `misb` (no dependencies of its own) |
| `cli` | yes | the `uas2cot` binary | `mavlink-tlog` (no dependencies of its own) |
| `mavlink-m` | no | `mavlink_m`: MAVLink-M friendly position reports, `mavlinkm-check` | `roxmltree` |

With `default-features = false` and no features you get the `Event` model,
the `mavlink` module (MAVLink `GLOBAL_POSITION_INT` and `HEARTBEAT`
payloads) and `net`, with no dependencies at all.

## The integration in five lines

```rust
use uas_cot::remoteid::{ble, Tracker};
use uas_cot::rid::{self, Options};

fn on_advertisement(tracker: &mut Tracker, mac: [u8; 6], adv: &[u8], now_ms: u64) -> Option<String> {
    let uas = tracker.ingest_ble(mac, &ble::find_in_advertising_data(adv)?, now_ms);
    Some(rid::aircraft(&uas.data, None, now_ms * 1000, &Options::default())?.to_xml())
}
```

The runnable version is [`examples/rid_to_cot.rs`](../examples/rid_to_cot.rs)
(`cargo run --example rid_to_cot`).

Use `uas_cot::remoteid` and `uas_cot::misb` to name those crates' types. They
are re-exports of the exact versions uas-cot is built against, so a
`Tracker` or `Packet` you create always fits `rid` and `fmv`, and Cargo
does not pull a second copy if you also depend on `remoteid` or `misb`
directly at a compatible version.

## The surface to depend on

| Item | What it is |
|---|---|
| `Event`, `Point`, `Track`, `Sensor`, `Link` | the CoT 2.0 event model; `Event::to_xml` writes it |
| `rid::aircraft`, `rid::operator`, `rid::Options` | Remote ID record to aircraft and operator markers |
| `fmv::events`, `fmv::platform`, `fmv::spi`, `fmv::Options` | ST 0601 packet to platform, field of view and sensor point of interest |
| `mavlink::vehicle`, `mavlink::GlobalPositionInt` | MAVLink position payload to a vehicle marker |
| `net::UdpSender`, `net::TcpSender`, `net::write_event` | SA multicast, TAK Server TCP input, any `io::Write` |

Everything returns `Option<Event>` or `Vec<Event>`: a record that cannot be
placed on the map (no position, invalid coordinates) gives `None`, never a
panic. Output is checked against the MITRE CoT schemas and converted through
`takproto` in CI ([testing.md](testing.md)).

## Compatibility

- MSRV: Rust 1.70, checked in CI. Edition 2021.
- License: MIT OR Apache-2.0.
- `#![forbid(unsafe_code)]`. CI runs on Linux, Windows and macOS.
- Versioning: while the crate is 0.x, a breaking change bumps the minor
  version (0.3 to 0.4) and is listed in [CHANGELOG.md](../CHANGELOG.md).
  Upgrading `remoteid` or `misb` across a breaking version is a breaking
  change here too, because their types are part of this API.
- Scope: civil telemetry and position interoperability. See
  [mavlink-m.md](mavlink-m.md) for what the optional MAVLink-M module
  refuses.

Maintained by Matt Busel ([github.com/Mattbusel](https://github.com/Mattbusel)),
who also maintains `remoteid`, `misb` and `mavlink-tlog` and keeps the
versions in step. Issues and pull requests:
[github.com/Mattbusel/uas-cot](https://github.com/Mattbusel/uas-cot/issues).
