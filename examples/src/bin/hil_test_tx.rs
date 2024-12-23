#![no_std]
#![no_main]
use core::marker::PhantomData;

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Ticker};
use esp32_wifi_hal_rs::{DMAResources, TxParameters, WiFi, WiFiRate};
use esp_backtrace as _;
use esp_hal::{efuse::Efuse, timer::timg::TimerGroup};
use esp_println::println;
use ieee80211::{
    common::{CapabilitiesInformation, SequenceControl, TU},
    element_chain,
    elements::{
        tim::{StaticBitmap, TIMBitmap, TIMElement},
        DSSSParameterSetElement,
    },
    mac_parser::MACAddress,
    mgmt_frame::{body::ProbeResponseBody, ManagementFrameHeader, ProbeResponseFrame},
    scroll::Pwrite,
    ssid, supported_rates,
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
const SSID: &str = "HIL";
// const BSSID: MACAddress = MACAddress::new([0x00, 0x80, 0x41, 0x13, 0x37, 0x69]);
#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
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
    println!("HIL test TX active.");
    let module_mac_address = Efuse::read_base_mac_address().into();
    let mut seq_num = 0;
    let start_timestamp = Instant::now();
    let mut beacon_ticker = Ticker::every(Duration::from_micros(TU.as_micros() as u64 * 100));
    loop {
        let mut buffer = [0u8; 1500];
        let frame = ProbeResponseFrame {
            header: ManagementFrameHeader {
                receiver_address: MACAddress::new([0x08, 0x3a, 0xf2, 0xa8, 0xc7, 0x73]),
                transmitter_address: module_mac_address,
                bssid: module_mac_address,
                sequence_control: SequenceControl::new().with_sequence_number(seq_num),
                ..Default::default()
            },
            body: ProbeResponseBody {
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
        let _ = wifi
            .transmit(
                &mut buffer[..written],
                &TxParameters {
                    rate: WiFiRate::PhyRate6M,
                    ..Default::default()
                },
            )
            .await;
        seq_num += 1;
        beacon_ticker.next().await;
    }
}
