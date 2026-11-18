//! The integration in one function: Bluetooth Remote ID advertising data in,
//! a CoT event out. `uas_cot::remoteid` is the exact `remoteid` version
//! uas-cot is built against, so the types always match.
//!
//! ```text
//! cargo run --example rid_to_cot
//! ```
use uas_cot::remoteid::{ble, BasicId, IdType, Location, Message, Tracker, UaType};
use uas_cot::rid::{self, Options};

/// Call this from your Bluetooth scanner callback (btleplug, BlueZ, an ESP32 bridge).
fn on_advertisement(
    tracker: &mut Tracker,
    mac: [u8; 6],
    adv: &[u8],
    now_ms: u64,
) -> Option<String> {
    let uas = tracker.ingest_ble(mac, &ble::find_in_advertising_data(adv)?, now_ms);
    // `tracked` adds the transmitter's trust verdict to the remarks.
    let events = rid::tracked(uas, None, now_ms * 1000, &Options::default());
    Some(events.first()?.to_xml())
}

fn main() -> Result<(), uas_cot::remoteid::Error> {
    // Two broadcasts as a transmitter would send them: Basic ID, then Location.
    let id = BasicId::new(
        IdType::SerialNumber,
        UaType::HelicopterOrMultirotor,
        "1596F35074529ABCDEFG",
    )?;
    let loc = Location {
        latitude: 45.5470818,
        longitude: -122.9668346,
        altitude_geo: Some(237.0),
        ..Default::default()
    };

    let mut tracker = Tracker::default();
    let mac = [0x02, 0x11, 0x22, 0x33, 0x44, 0x55];
    let mut adv = [0u8; 31];
    for (counter, msg) in [Message::BasicId(id), Message::Location(loc)]
        .iter()
        .enumerate()
    {
        ble::write_advertising_data(counter as u8, &msg.encode()?, &mut adv)?;
        if let Some(xml) =
            on_advertisement(&mut tracker, mac, &adv, 1_700_000_000_000 + counter as u64)
        {
            println!("{xml}");
        }
    }
    Ok(())
}
