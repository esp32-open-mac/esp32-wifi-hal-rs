#![no_std]
#![no_main]

extern crate alloc;

use core::{iter::repeat, mem::MaybeUninit};

use alloc::{
    collections::btree_set::BTreeSet,
    string::{String, ToString},
};
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Ticker};
use esp32_wifi_hal_rs::WiFi;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;
use ieee80211::{match_frames, mgmt_frame::BeaconFrame, GenericFrame};
use log::{info, LevelFilter};

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            HEAP.as_mut_ptr() as *mut u8,
            HEAP_SIZE,
            esp_alloc::MemoryCapability::Internal.into(),
        ));
    }
}

async fn scan_on_channel(wifi: &mut WiFi, known_ssids: &mut BTreeSet<String>) {
    let received = wifi.receive().await;
    let buffer = received.mpdu_buffer();
    let res = match_frames! {
        buffer,
        beacon_frame = BeaconFrame => {
            let ssid = beacon_frame.ssid().unwrap_or_default();
            if known_ssids.insert(ssid.to_string()) {
                info!("Found new AP with SSID: {ssid}");
            }
        }
    };
    if res.is_err() {
        let generic_frame = GenericFrame::new(buffer, false).unwrap();
        info!(
            "Got non beacon frame of type: {:?}",
            generic_frame.frame_control_field().frame_type()
        );
    }
}

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    init_heap();
    esp_println::logger::init_logger(LevelFilter::Info);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let mut wifi = WiFi::new(peripherals.WIFI, peripherals.RADIO_CLK, peripherals.ADC2);
    let mut known_ssids = BTreeSet::new();
    let mut hop_set = repeat([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]).flatten();
    let mut hop_interval = Ticker::every(Duration::from_secs(1));
    loop {
        if let Either::Second(_) = select(
            scan_on_channel(&mut wifi, &mut known_ssids),
            hop_interval.next(),
        )
        .await
        {
            let _ = wifi.set_channel(hop_set.next().unwrap());
        }
    }
}
