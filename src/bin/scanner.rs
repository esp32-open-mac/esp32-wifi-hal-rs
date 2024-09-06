#![no_std]
#![no_main]

extern crate alloc;

use core::{iter::repeat, mem::MaybeUninit};

use alloc::{collections::btree_set::BTreeSet, string::ToString};
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Ticker};
use esp32_open_mac_rs::WiFi;
use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;
use esp_wifi_sys::include::wifi_pkt_rx_ctrl_t;
use ieee80211::{match_frames, mgmt_frame::BeaconFrame};
use log::info;

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
    esp_println::logger::init_logger_from_env();

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let wifi = WiFi::new(peripherals.WIFI, peripherals.RADIO_CLK, peripherals.ADC2);
    let mut known_ssids = BTreeSet::new();
    let mut hop_set = repeat([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]).flatten();
    let mut hop_interval = Ticker::every(Duration::from_secs(1));
    loop {
        match select(wifi.receive(), hop_interval.next()).await {
            Either::First(received) => {
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
            Either::Second(_) => {
                wifi.change_channel(hop_set.next().unwrap());
            }
        }
    }
}
