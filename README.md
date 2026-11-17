# uas-cot

[![crates.io](https://img.shields.io/crates/v/uas-cot.svg)](https://crates.io/crates/uas-cot)
[![docs.rs](https://docs.rs/uas-cot/badge.svg)](https://docs.rs/uas-cot)
[![CI](https://github.com/Mattbusel/uas-cot/actions/workflows/ci.yml/badge.svg)](https://github.com/Mattbusel/uas-cot/actions/workflows/ci.yml)

Put drones on the TAK map. Remote ID, MISB KLV video metadata and MAVLink to Cursor on Target.

- Remote ID (Wi-Fi Beacon, NAN, Bluetooth via [`remoteid`](https://crates.io/crates/remoteid)): aircraft and operator markers
- MISB ST 0601 KLV from STANAG 4609 video (via [`misb`](https://crates.io/crates/misb)): platform, camera field of view, sensor point of interest
- MAVLink `GLOBAL_POSITION_INT`: vehicle markers, from a `.tlog` or live UDP
- Sends to the SA multicast group (239.2.3.1:6969), a TAK Server TCP input, or stdout
- Output passes the MITRE CoT schemas and converts cleanly to TAK Protocol v1

## Install

```
cargo install uas-cot
```

## Use

```
uas2cot pcap capture.pcap --udp 239.2.3.1:6969        # Remote ID from a Wi-Fi capture
uas2cot klv flight.klv --video rtsp://cam/1 --realtime   # ffmpeg -i in.ts -map 0:d -c copy -f data flight.klv
uas2cot tlog flight.tlog --tcp takserver:8087
uas2cot mavlink-udp 0.0.0.0:14550 --udp 239.2.3.1:6969
```

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
- [docs/tak.md](docs/tak.md): seeing it in ATAK, WinTAK and TAK Server
- [docs/testing.md](docs/testing.md): how it is checked
- API: [docs.rs/uas-cot](https://docs.rs/uas-cot)

## License

MIT OR Apache-2.0.
