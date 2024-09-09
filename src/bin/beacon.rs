#![no_main]
#![no_std]
use core::{marker::PhantomData, mem::MaybeUninit};

use embassy_executor::Spawner;
use embassy_time::{Instant, Timer};
use esp32_open_mac_rs::{WiFi, WiFiRate};
use esp_backtrace as _;
use esp_hal::{efuse::Efuse, timer::timg::TimerGroup};
use esp_hal_embassy::main;
use ieee80211::{
    common::{CapabilitiesInformation, SequenceControl},
    element_chain,
    elements::{
        tim::{StaticBitmap, TIMBitmap, TIMElement},
        DSSSParameterSetElement,
    },
    mac_parser::BROADCAST,
    mgmt_frame::{body::BeaconBody, BeaconFrame, ManagementFrameHeader},
    scroll::Pwrite,
    ssid, supported_rates,
};

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

const SSID: &str = "The cake is a lie";

#[main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    init_heap();
    esp_println::logger::init_logger_from_env();

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let wifi = WiFi::new(peripherals.WIFI, peripherals.RADIO_CLK, peripherals.ADC2);
    let start_timestamp = Instant::now();
    let module_mac_address = Efuse::read_base_mac_address().into();
    let mut seq_num = 0;
    loop {
        let mut buffer = [0u8; 1500];
        let frame = BeaconFrame {
            header: ManagementFrameHeader {
                receiver_address: BROADCAST,
                transmitter_address: module_mac_address,
                bssid: module_mac_address,
                sequence_control: SequenceControl::new().with_sequence_number(seq_num),
                ..Default::default()
            },
            body: BeaconBody {
                beacon_interval: 100,
                timestamp: start_timestamp.elapsed().as_micros(),
                capabilities_info: CapabilitiesInformation::new().with_is_ess(true),
                elements: element_chain! {
                    ssid!(SSID),
                    supported_rates![
                            1 B,
                            2 B,
                            5.5 B,
                            11 B,
                            6,
                            9,
                            12,
                            18
                        ],
                    DSSSParameterSetElement {
                        current_channel: 1
                    },
                    TIMElement {
                        dtim_count: 1,
                        dtim_period: 2,
                        bitmap: None::<TIMBitmap<StaticBitmap>>,
                        _phantom: PhantomData
                    }
                },
                ..Default::default()
            },
        };
        let written = buffer.pwrite(frame, 0).unwrap();
        wifi.transmit(&buffer[..written], WiFiRate::PhyRate6M).await;
        Timer::after_millis(100).await;
        seq_num += 1;
    }
}
