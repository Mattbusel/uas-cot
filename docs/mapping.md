# Field mapping

All events are CoT 2.0 (`<event version="2.0">`) with `<point>` and
`<detail>`. Times are UTC with millisecond precision.

## Remote ID (`rid`)

| CoT | From |
|---|---|
| `uid` | `rid-` + first Basic ID (serial number, registration ...), else the radio address |
| `type` | `a-u-A-M-F-Q` (drone) for powered aircraft, `a-u-A-C-L` for balloons and airships, `a-u-A` otherwise. `u` is the affiliation (option) |
| `point lat lon` | Location message |
| `point hae` | geodetic altitude, else pressure altitude (`precisionlocation altsrc` says which) |
| `point ce le` | horizontal and vertical accuracy, metres; unknown is 9999999 |
| `track course speed` | direction, horizontal speed |
| `contact callsign` | the Basic ID |
| `remarks` | UA type, status, height and its reference, vertical speed, operator ID, self ID |
| stale | 30 s |

The operator is a second event: `uid` `rid-op-<id>`, type `b-m-p-s-m`
(spot marker, as DragonSync uses), at the System message operator position.

## MISB ST 0601 (`fmv`)

| CoT | From (ST 0601 tag) |
|---|---|
| `uid` | `fmv-` + tail number (4), call sign (59), designation (10) or mission ID (3); `fmv-UAS` if none; or the `--uid` option |
| `type` | `a-f-A-M-F-Q`; `f` is the affiliation (option) |
| `time` | Precision Time Stamp (2) |
| `point lat lon` | sensor latitude, longitude (13, 14) |
| `point hae` | sensor ellipsoid height (104, else 75); else true altitude MSL (15) |
| `track course speed` | platform course (112, else heading 5), ground speed (56) |
| `sensor azimuth` | platform heading (5) + sensor relative azimuth (18) |
| `sensor elevation roll` | sensor relative elevation (19), relative roll (20) |
| `sensor fov vfov range model` | fields of view (16, 17), slant range (21), image source sensor (11) |
| `__video url` | the `--video` option |

The sensor point of interest is a second event: `uid` `<platform uid>.SPI1`,
type `b-m-p-s-p-i`, how `m-i`, at the frame center (23, 24; height 78, else
25), `ce` from CE90 (45) / 2.146, with `<link relation="p-p">` to the platform.

## MAVLink (`mavlink`)

| CoT | From |
|---|---|
| `uid` | `mav-` + system id |
| `type` | from `HEARTBEAT.type`: aircraft `a-f-A-M-F-Q`, rover `a-f-G-E-V`, boat `a-f-S`, submarine `a-f-U`, unknown `a-f-A` |
| `point` | `GLOBAL_POSITION_INT` lat, lon (1e-7 deg), alt (mm, MSL) |
| `track` | `hdg` (cdeg) when known, else the direction of `vx`, `vy`; speed from `vx`, `vy` |
| `remarks` | height above home (`relative_alt`) |
| stale | 15 s |

## MAVLink-M (`mavlink_m`, feature `mavlink-m`)

Friendly position reports (`PARTICIPANT_POSITION`) only. Target, track,
handover, fires, munition and engagement messages are refused. See
[mavlink-m.md](mavlink-m.md) for the scope, the table and the conformance
checks.

## Heights

CoT `hae` is height above the WGS84 ellipsoid. When a source only gives
MSL altitude (ST 0601 tag 15, MAVLink `alt`), that value is used as is
and `precisionlocation altsrc="MSL"` marks it; the geoid offset (up to
about 100 m) is not applied.
