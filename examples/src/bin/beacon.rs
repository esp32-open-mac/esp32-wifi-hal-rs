#![no_main]
#![no_std]
use core::marker::PhantomData;

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Ticker};
use esp32_wifi_hal_rs::{DMAResources, TxParameters, WiFi, WiFiRate};
use esp_backtrace as _;
use esp_hal::{efuse::Efuse, timer::timg::TimerGroup};
use esp_hal_embassy::main;
use ieee80211::{
    common::{CapabilitiesInformation, SequenceControl, TU},
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
use log::LevelFilter;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

const SSID: &str = "The cake is a lie.";

#[main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_println::logger::init_logger(LevelFilter::Debug);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let dma_resources = mk_static!(DMAResources<1500, 10>, DMAResources::new());

    let wifi = WiFi::new(
        peripherals.WIFI,
        peripherals.RADIO_CLK,
        peripherals.ADC2,
        dma_resources,
    );
    let module_mac_address = Efuse::read_base_mac_address().into();
    let mut beacon_ticker = Ticker::every(Duration::from_micros(TU.as_micros() as u64 * 100));
    let start_timestamp = Instant::now();
    let mut buffer = [0u8; 300];
    let frame = BeaconFrame {
        header: ManagementFrameHeader {
            receiver_address: BROADCAST,
            transmitter_address: module_mac_address,
            bssid: module_mac_address,
            sequence_control: SequenceControl::new(),
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
    loop {
        wifi.transmit(
            &mut buffer[..written],
            &TxParameters {
                rate: WiFiRate::PhyRate1ML,
                override_seq_num: true,
                ..Default::default()
            },
        )
        .await
        .unwrap();
        beacon_ticker.next().await;
    }
}
