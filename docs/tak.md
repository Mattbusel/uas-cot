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

TLS streaming input (TAK Server default 8089, client certificate
required), with `cargo install uas-cot --features tls`:

```
openssl pkcs12 -legacy -in truststore-root.p12 -nokeys -out ca.pem
openssl pkcs12 -legacy -in user.p12 -clcerts -nokeys -out user.pem
openssl pkcs12 -legacy -in user.p12 -nocerts -nodes -out user.key
uas2cot pcap capture.pcap --tls takserver:8089 --ca ca.pem --cert user.pem --key user.key
```

The server certificate must name the host you dial. rustls does not fall
back to the certificate's common name, so a certificate with only a CN, or
a server reached by an IP address its certificate does not list, fails the
name check. `--tls-any-name` (`TlsConfig::from_pem_any_name` in the
library) accepts it when the chain verifies against `ca.pem`; an unknown or
expired certificate still fails.

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
