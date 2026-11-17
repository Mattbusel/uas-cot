# Seeing the events in TAK

## ATAK, iTAK, WinTAK on the same network

TAK clients listen to the SA multicast group `239.2.3.1:6969` by default.

```
uas2cot pcap capture.pcap --udp 239.2.3.1:6969 --realtime
```

The multicast TTL is 1, so it stays on the local network. If nothing
shows up, check that the client has "Multicast" input enabled and that
the sending machine's firewall and Wi-Fi allow multicast.

## TAK Server, FreeTAKServer, OpenTAKServer

Plain TCP streaming input (TAK Server default 8087, FreeTAKServer 8087):

```
uas2cot tlog flight.tlog --tcp takserver:8087
```

For a TLS input (8089), use the library: open your TLS stream and call
`uas_cot::net::write_event` for each event.

## Live sources

- Remote ID: put a Wi-Fi card in monitor mode on channel 6 and capture
  with `tcpdump -i wlan0mon -w rid.pcap`, or use the library with your own
  receiver (`remoteid::Tracker` + `uas_cot::rid::events`).
- MAVLink: `uas2cot mavlink-udp 0.0.0.0:14550 --udp 239.2.3.1:6969`, with
  the autopilot or MAVProxy forwarding to that port.
- Video KLV: `ffmpeg -i udp://@:5000 -map 0:d -c copy -f data -` gives the
  KLV stream; for live use, read it with `misb::klv::split` and
  `uas_cot::fmv::events` in your own loop.

## Markers

- Remote ID aircraft: unknown air drone (`a-u-A-M-F-Q`); the operator as a
  spot marker. Change the affiliation with `--affiliation`.
- FMV platform: friendly air drone with a sensor field of view; the sensor
  point of interest marker linked to it. `--video` adds the stream to the
  marker so the client can open it.
