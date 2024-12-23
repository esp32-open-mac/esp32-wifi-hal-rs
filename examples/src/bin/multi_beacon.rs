//! This example demonstrates, that concurrent tranmissions are possible.

#![no_main]
#![no_std]
use core::marker::PhantomData;

use embassy_executor::Spawner;
use embassy_time::{Instant, Timer};
use esp32_wifi_hal_rs::{DMAResources, TxParameters, WiFi, WiFiRate};
use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;
use esp_hal_embassy::main;
use ieee80211::{
    common::{CapabilitiesInformation, SequenceControl},
    element_chain,
    elements::{
        tim::{StaticBitmap, TIMBitmap, TIMElement},
        DSSSParameterSetElement, SSIDElement,
    },
    mac_parser::{MACAddress, BROADCAST},
    mgmt_frame::{body::BeaconBody, BeaconFrame, ManagementFrameHeader},
    scroll::Pwrite,
    supported_rates,
};
use log::LevelFilter;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

static SSIDS: [&str; 6] = [
    "Never gonna give you up",
    "Never gonna let you down",
    "Never gonna run around",
    "Never gonna make you cry",
    "Never gonna say goodbye",
    "Never gonna tell a lie",
];

#[embassy_executor::task(pool_size = 6)]
async fn beacon_task(ssid: &'static str, id: u8, wifi: &'static WiFi<'static>) {
    let start_timestamp = Instant::now();
    let mut seq_num = 0;
    let mac_address = MACAddress::new([0x00, 0x80, 0x41, 0x13, 0x37, id]);
    loop {
        let mut buffer = [0u8; 1500];
        let frame = BeaconFrame {
            header: ManagementFrameHeader {
                receiver_address: BROADCAST,
                transmitter_address: mac_address,
                bssid: mac_address,
                sequence_control: SequenceControl::new().with_sequence_number(seq_num),
                ..Default::default()
            },
            body: BeaconBody {
                beacon_interval: 100,
                timestamp: start_timestamp.elapsed().as_micros(),
                capabilities_info: CapabilitiesInformation::new().with_is_ess(true),
                elements: element_chain! {
                    SSIDElement::new(ssid).unwrap(),
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
        let _ = wifi
            .transmit(
                &mut buffer[..written],
                &TxParameters {
                    rate: WiFiRate::PhyRate6M,
                    ..Default::default()
                },
            )
            .await;
        Timer::after_millis(100).await;
        seq_num += 1;
    }
}

#[main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_println::logger::init_logger(LevelFilter::Info);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let dma_resources = mk_static!(DMAResources<1500, 10>, DMAResources::new());
    let wifi = mk_static!(
        WiFi,
        WiFi::new(
            peripherals.WIFI,
            peripherals.RADIO_CLK,
            peripherals.ADC2,
            dma_resources,
        )
    );
    for (id, ssid) in SSIDS.iter().enumerate() {
        spawner.spawn(beacon_task(ssid, id as u8, wifi)).unwrap();
    }
}
