#![no_std]
#![no_main]

extern crate alloc;

use core::mem::MaybeUninit;

use embassy_executor::Spawner;
use esp32_wifi_hal_rs::{DMAResources, RxFilterBank, RxFilterInterface, WiFi};
use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;
use ieee80211::GenericFrame;
use log::{info, LevelFilter};

use esp_alloc as _;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}
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

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    init_heap();
    esp_println::logger::init_logger(LevelFilter::Info);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let dma_resources = mk_static!(DMAResources<1500, 10>, DMAResources::new());
    let wifi = WiFi::new(
        peripherals.WIFI,
        peripherals.RADIO_CLK,
        peripherals.ADC2,
        dma_resources,
    );
    let _ = wifi.set_channel(1);
    /* wifi.set_filter(
        RxFilterBank::BSSID,
        RxFilterInterface::Zero,
        [0x1c, 0xed, 0x6f, 0x3a, 0x20, 0x2a],
        // [0xFC, 0xEC, 0xDA, 0x87, 0xD7, 0x30],
        [0x00; 6],
    );*/
    wifi.set_filter_status(RxFilterBank::BSSID, RxFilterInterface::Zero, false);
    /* wifi.set_filter(
        RxFilterBank::ReceiverAddress,
        RxFilterInterface::Zero,
        [0x00; 6],
        [0x00; 6],
    );*/
    wifi.set_filter_status(
        RxFilterBank::ReceiverAddress,
        RxFilterInterface::Zero,
        false,
    );
    // wifi.set_scanning_mode(RxFilterInterface::Zero, true);
    loop {
        let received = wifi.receive().await;
        let buffer = received.mpdu_buffer();
        let generic_frame = GenericFrame::new(buffer, false).unwrap();
        info!(
            "Address1: {} Address2: {:?} Type {:?}",
            generic_frame.address_1(),
            generic_frame.address_2(),
            generic_frame.frame_control_field().frame_type()
        );
        /*let _ = match_frames! {
            buffer,
            beacon_frame = BeaconFrame => {
                info!("SSID: {}", beacon_frame.ssid().unwrap());
            }
        };*/
    }
}
