#![no_std]
#![no_main]
use core::mem::MaybeUninit;

use embassy_executor::Spawner;
use embassy_time::Instant;
use esp32_wifi_hal_rs::{DMAResources, RxFilterBank, RxFilterInterface, WiFi};
use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;
use ieee80211::{common::TU, match_frames, mgmt_frame::BeaconFrame};
use log::LevelFilter;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

use esp_alloc as _;

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
const TARGET_SSID: &str = "HIL";
#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    init_heap();
    esp_println::logger::init_logger(LevelFilter::Warn);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let dma_resources = mk_static!(DMAResources<1500, 10>, DMAResources::new());
    let mut wifi = WiFi::new(
        peripherals.WIFI,
        peripherals.RADIO_CLK,
        peripherals.ADC2,
        dma_resources,
    );
    wifi.set_filter_status(RxFilterBank::BSSID, RxFilterInterface::Zero, true);
    wifi.set_filter_status(RxFilterBank::ReceiverAddress, RxFilterInterface::Zero, true);
    wifi.set_scanning_mode(RxFilterInterface::Zero, true);
    println!("HIL test RX active.");
    let mut last_beacon_received = Instant::now();
    loop {
        let received = wifi.receive().await;
        let _ = match_frames! {
            received.mpdu_buffer(),
            beacon_frame = BeaconFrame => {
                // println!("RX'd beacon, with SSID: {}", beacon_frame.ssid().unwrap_or_default());
                let ssid = beacon_frame.ssid();
                if ssid == Some(TARGET_SSID) {
                    println!("RX'd beacon from {TARGET_SSID}. Last beacon received: {}TUs. Sequence number: {}", (last_beacon_received.elapsed().as_micros() / TU.as_micros() as u64), beacon_frame.header.sequence_control.sequence_number());
                    last_beacon_received = Instant::now();
                }
            }
        };
    }
}
