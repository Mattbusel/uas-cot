# uas-cot

[![crates.io](https://img.shields.io/crates/v/uas-cot.svg)](https://crates.io/crates/uas-cot)
[![docs.rs](https://docs.rs/uas-cot/badge.svg)](https://docs.rs/uas-cot)
[![CI](https://github.com/Mattbusel/uas-cot/actions/workflows/ci.yml/badge.svg)](https://github.com/Mattbusel/uas-cot/actions/workflows/ci.yml)

Put drones on the TAK map. Remote ID, MISB KLV video metadata, MAVLink and MAVLink-M to Cursor on Target.

- Remote ID (Wi-Fi Beacon, NAN, Bluetooth via [`remoteid`](https://crates.io/crates/remoteid)): aircraft and operator markers
- MISB ST 0601 KLV from STANAG 4609 video (via [`misb`](https://crates.io/crates/misb)): platform, camera field of view, sensor point of interest
- MAVLink `GLOBAL_POSITION_INT`: vehicle markers, from a `.tlog` or live UDP
- MAVLink-M (Dronecode `military.xml`, feature `mavlink-m`): shared tracks (`TRACK_IDENTITY`, `TARGET_HANDOVER`, `TARGET`) and friendly position reports (`PARTICIPANT_POSITION`) as CoT markers, plus `mavlinkm-check`, a conformance checker for MAVLink-M traffic. See [docs/mavlink-m.md](docs/mavlink-m.md)
- Sends to the SA multicast group (239.2.3.1:6969), a TAK Server TCP input, or stdout
- Output passes the MITRE CoT schemas and converts cleanly to TAK Protocol v1

## Install

```
cargo install uas-cot                       # Remote ID, KLV, MAVLink
cargo install uas-cot --features mavlink-m  # plus MAVLink-M and mavlinkm-check
```

## Use

```
uas2cot pcap capture.pcap --udp 239.2.3.1:6969        # Remote ID from a Wi-Fi capture
uas2cot klv flight.klv --video rtsp://cam/1 --realtime   # ffmpeg -i in.ts -map 0:d -c copy -f data flight.klv
uas2cot tlog flight.tlog --tcp takserver:8087
uas2cot mavlink-udp 0.0.0.0:14550 --udp 239.2.3.1:6969   # with mavlink-m: MAVLink-M tracks too
mavlinkm-check tlog flight.tlog                           # MAVLink-M traffic against military.xml
```

![MAVLink + MAVLink-M to CoT, simulated scenario](https://raw.githubusercontent.com/Mattbusel/uas-cot/master/docs/mavlink-m-demo.gif)

*uas2cot output for the simulated scenario in `examples/mavlink_m_sim.rs`
(synthetic data): two drones, a friendly ground team relayed as a MAVLink-M
PPLI report, and a vehicle track handed from one drone to the other.*

As a library:

```rust
use uas_cot::{mavlink, net::UdpSender};

let pos = mavlink::GlobalPositionInt { lat: 473_977_418, lon: 85_455_939, alt: 488_000, ..Default::default() };
if let Some(event) = mavlink::vehicle(1, &pos, Some(2), uas_cot::now_us(), &Default::default()) {
    UdpSender::sa_multicast()?.send(&event)?;
}
# Ok::<(), std::io::Error>(())
```

## Docs

- [docs/mapping.md](docs/mapping.md): which field goes where
- [docs/mavlink-m.md](docs/mavlink-m.md): MAVLink-M mapping, what is never translated, the conformance checks, the demo
- [docs/tak.md](docs/tak.md): seeing it in ATAK, WinTAK and TAK Server
- [docs/testing.md](docs/testing.md): how it is checked
- API: [docs.rs/uas-cot](https://docs.rs/uas-cot)

## License

MIT OR Apache-2.0. `dialects/military.xml` is from
[Dronecode/mavlink-military](https://github.com/Dronecode/mavlink-military),
MIT, Copyright (c) 2026 Dronecode Project, Inc (`dialects/LICENSE-MIT-dronecode`).
