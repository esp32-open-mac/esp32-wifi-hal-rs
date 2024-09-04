#![no_std]
#![no_main]
use core::mem::MaybeUninit;

use alloc::{collections::btree_set::BTreeSet, string::ToString};
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_wifi_sys::include::wifi_pkt_rx_ctrl_t;
use ieee80211::{match_frames, mgmt_frame::BeaconFrame};
use log::{info, LevelFilter};
use wmac::WiFi;

extern crate alloc;

mod dma_list;
mod ffi;
mod phy_init_data;
mod regs;
mod wmac;

/* macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
} */

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    init_heap();
    esp_println::logger::init_logger(LevelFilter::Debug);

    let wifi = WiFi::new(peripherals.WIFI, peripherals.RADIO_CLK, peripherals.ADC2);
    let mut known_ssids = BTreeSet::new();
    loop {
        let received = wifi.receive().await;
        let buffer = &received[size_of::<wifi_pkt_rx_ctrl_t>()..];
        let _ = match_frames! {
            buffer,
            beacon_frame = BeaconFrame => {
                let ssid = beacon_frame.ssid().unwrap_or_default();
                if known_ssids.insert(ssid.to_string()) {
                    info!("Found new AP with SSID: {ssid}");
                }
            }
        };
    }
}