//! A scripted MAVLink + MAVLink-M emitter for trying the gateway without
//! hardware. Every message it sends is synthetic, around the PX4 SITL
//! default home (47.397742, 8.545594).
//!
//! Two friendly aircraft (system 1, a quadrotor; system 2, a fixed wing)
//! send HEARTBEAT and GLOBAL_POSITION_INT. System 1 relays a friendly
//! ground team (system 3) and system 2 relays a friendly survey boat
//! (system 4) with MAVLink-M PARTICIPANT_POSITION. That is all: the
//! scenario is position and telemetry interoperability, and uas-cot
//! refuses every target, track, fires or engagement message.
//!
//! ```text
//! cargo run --example mavlink_m_sim --features mavlink-m -- --tlog demo.tlog [--secs 60] [--fault]
//! cargo run --example mavlink_m_sim --features mavlink-m -- --udp 127.0.0.1:14550 [--secs 60] [--fault]
//! ```
//!
//! `--fault` also sends one PARTICIPANT_POSITION whose `ppli_type` holds a
//! value the dialect does not define, for the conformance checker to catch.

use mavlink_tlog::{encode, Header, TlogWriter, Version};
use std::io;
use std::time::{Duration, Instant};
use uas_cot::mavlink_m::{self, Value};

const HOME: (f64, f64) = (47.397742, 8.545594);
const LOG_T0_US: u64 = 1_763_400_000_000_000; // 2025-11-17T17:20:00Z, a fixed start so logs are reproducible

/// Offsets in metres to degrees around HOME.
fn geo(north: f64, east: f64) -> (f64, f64) {
    let lat = HOME.0 + north / 111_320.0;
    let lon = HOME.1 + east / (111_320.0 * HOME.0.to_radians().cos());
    (lat, lon)
}

struct Out {
    seq: u8,
    frames: Vec<Vec<u8>>,
}

impl Out {
    fn push(&mut self, sys: u8, msg_id: u32, payload: &[u8], crc_extra: u8) {
        let mut buf = [0u8; 300];
        let n = encode(
            Version::V2,
            Header {
                seq: self.seq,
                sys_id: sys,
                comp_id: 1,
                msg_id,
            },
            payload,
            Some(crc_extra),
            &mut buf,
        )
        .expect("frame fits");
        self.seq = self.seq.wrapping_add(1);
        self.frames.push(buf[..n].to_vec());
    }

    fn m(&mut self, sys: u8, name: &str, values: &[(&str, Value)]) {
        let d = mavlink_m::military();
        let m = d.message_by_name(name).expect("message in military.xml");
        let p = m.encode(values).expect("valid values");
        self.push(sys, m.id, &p, m.crc_extra);
    }
}

fn heartbeat(mav_type: u8) -> Vec<u8> {
    // custom_mode u32, type, autopilot (12 = PX4), base_mode, system_status (4 = active), mavlink_version 3
    let mut p = vec![0u8; 4];
    p.extend_from_slice(&[mav_type, 12, 0x81, 4, 3]);
    p
}

fn global_position_int(
    t_ms: u32,
    lat: f64,
    lon: f64,
    alt_m: f64,
    rel_m: f64,
    vn: f64,
    ve: f64,
) -> Vec<u8> {
    let mut p = Vec::with_capacity(28);
    p.extend_from_slice(&t_ms.to_le_bytes());
    p.extend_from_slice(&((lat * 1e7).round() as i32).to_le_bytes());
    p.extend_from_slice(&((lon * 1e7).round() as i32).to_le_bytes());
    p.extend_from_slice(&((alt_m * 1000.0) as i32).to_le_bytes());
    p.extend_from_slice(&((rel_m * 1000.0) as i32).to_le_bytes());
    p.extend_from_slice(&((vn * 100.0) as i16).to_le_bytes());
    p.extend_from_slice(&((ve * 100.0) as i16).to_le_bytes());
    p.extend_from_slice(&0i16.to_le_bytes());
    let hdg = (ve.atan2(vn).to_degrees().rem_euclid(360.0) * 100.0) as u16;
    p.extend_from_slice(&hdg.to_le_bytes());
    p
}

fn deg_e7(v: f64) -> Value {
    Value::Int((v * 1e7).round() as i64)
}

/// All frames for time `t` seconds into the scenario.
fn step(out: &mut Out, t0_us: u64, t: f64, fault: bool) {
    let now_us = t0_us + (t * 1e6) as u64;
    let t_ms = (t * 1000.0) as u32;
    let ground_alt = 488.0;

    // system 1: quadrotor orbiting home at 300 m radius
    let w = 2.0 * std::f64::consts::PI / 90.0;
    let (n1, e1) = (300.0 * (w * t).cos(), 300.0 * (w * t).sin());
    let (vn1, ve1) = (-300.0 * w * (w * t).sin(), 300.0 * w * (w * t).cos());
    let (lat1, lon1) = geo(n1, e1);
    // system 2: fixed wing on a long east-west racetrack 1.5 km north
    let x2 = 1200.0 * (2.0 * std::f64::consts::PI * t / 120.0).sin();
    let ve2 = 1200.0 * 2.0 * std::f64::consts::PI / 120.0
        * (2.0 * std::f64::consts::PI * t / 120.0).cos();
    let (lat2, lon2) = geo(1500.0, x2);

    if (t * 10.0).round() as i64 % 10 == 0 {
        out.push(1, 0, &heartbeat(2), 50);
        out.push(2, 0, &heartbeat(1), 50);
    }
    out.push(
        1,
        33,
        &global_position_int(t_ms, lat1, lon1, ground_alt + 120.0, 120.0, vn1, ve1),
        104,
    );
    out.push(
        2,
        33,
        &global_position_int(t_ms, lat2, lon2, ground_alt + 400.0, 400.0, 0.0, ve2),
        104,
    );

    if (t * 10.0).round() as i64 % 10 == 0 {
        // a friendly ground team with no MAVLink telemetry of its own,
        // relayed by system 1 as a PPLI-style report
        let (ng, eg) = (-650.0 + 1.2 * t, 250.0);
        let (latg, long) = geo(ng, eg);
        out.m(
            1,
            "PARTICIPANT_POSITION",
            &[
                ("time_usec", Value::Uint(now_us)),
                ("lat", deg_e7(latg)),
                ("lon", deg_e7(long)),
                ("alt", Value::Float(ground_alt)),
                ("vx", Value::Float(1.2)),
                ("vy", Value::Float(0.0)),
                ("vz", Value::Float(0.0)),
                ("course", Value::Float(f64::NAN)),
                ("callsign", Value::Text("ANVIL 3".into())),
                ("origin_sysid", Value::Uint(3)),
                ("stanag_identity", Value::Uint(3)), // FRIEND
                ("ppli_type", Value::Uint(2)),       // LAND
            ],
        );
    }

    if (t * 10.0).round() as i64 % 10 == 5 {
        // a friendly survey boat 1.8 km north-east (synthetic),
        // relayed by system 2
        let (nb, eb) = (1800.0, 900.0 - 2.0 * t);
        let (latb, lonb) = geo(nb, eb);
        out.m(
            2,
            "PARTICIPANT_POSITION",
            &[
                ("time_usec", Value::Uint(now_us)),
                ("lat", deg_e7(latb)),
                ("lon", deg_e7(lonb)),
                ("alt", Value::Float(406.0)),
                ("vx", Value::Float(0.0)),
                ("vy", Value::Float(-2.0)),
                ("vz", Value::Float(0.0)),
                ("course", Value::Float(f64::NAN)),
                ("callsign", Value::Text("HERON 4".into())),
                ("origin_sysid", Value::Uint(4)),
                ("stanag_identity", Value::Uint(3)), // FRIEND
                ("ppli_type", Value::Uint(3)),       // SURFACE
            ],
        );
    }
    if fault && (t * 10.0).round() as i64 == 125 {
        let (latg, long) = geo(-650.0 + 1.2 * t, 250.0);
        out.m(
            1,
            "PARTICIPANT_POSITION",
            &[
                ("time_usec", Value::Uint(now_us)),
                ("lat", deg_e7(latg)),
                ("lon", deg_e7(long)),
                ("alt", Value::Float(ground_alt)),
                ("callsign", Value::Text("ANVIL 3".into())),
                ("origin_sysid", Value::Uint(3)),
                ("stanag_identity", Value::Uint(3)),
                ("ppli_type", Value::Uint(9)), // not defined in MAVLINK_M_PPLI_TYPE
            ],
        );
    }
}

fn main() -> io::Result<()> {
    let mut args = std::env::args().skip(1);
    let mut tlog = None;
    let mut udp = None;
    let mut secs = 60.0;
    let mut fault = false;
    while let Some(a) = args.next() {
        match a.as_str() {
            "--tlog" => tlog = args.next(),
            "--udp" => udp = args.next(),
            "--secs" => secs = args.next().and_then(|v| v.parse().ok()).unwrap_or(60.0),
            "--fault" => fault = true,
            _ => {
                eprintln!(
                    "usage: mavlink_m_sim (--tlog out.tlog | --udp host:port) [--secs n] [--fault]"
                );
                std::process::exit(2);
            }
        }
    }
    let mut out = Out {
        seq: 0,
        frames: Vec::new(),
    };
    let ticks = (secs * 10.0) as u64;
    if let Some(path) = tlog {
        let mut w = TlogWriter::new(io::BufWriter::new(std::fs::File::create(&path)?));
        for k in 0..ticks {
            let t = k as f64 / 10.0;
            step(&mut out, LOG_T0_US, t, fault);
            for f in out.frames.drain(..) {
                w.write_frame(LOG_T0_US + k * 100_000, &f)?;
            }
        }
        w.flush()?;
        eprintln!("mavlink_m_sim: wrote {} records to {path}", w.records());
    } else if let Some(dest) = udp {
        let sock = std::net::UdpSocket::bind("0.0.0.0:0")?;
        let start = Instant::now();
        let t0_us = uas_cot::now_us();
        let mut sent = 0;
        for k in 0..ticks {
            let t = k as f64 / 10.0;
            step(&mut out, t0_us, t, fault);
            for f in out.frames.drain(..) {
                sock.send_to(&f, &dest)?;
                sent += 1;
            }
            let due = start + Duration::from_millis(k * 100 + 100);
            if let Some(d) = due.checked_duration_since(Instant::now()) {
                std::thread::sleep(d);
            }
        }
        eprintln!("mavlink_m_sim: sent {sent} frames to {dest}");
    } else {
        eprintln!("usage: mavlink_m_sim (--tlog out.tlog | --udp host:port) [--secs n] [--fault]");
        std::process::exit(2);
    }
    Ok(())
}
