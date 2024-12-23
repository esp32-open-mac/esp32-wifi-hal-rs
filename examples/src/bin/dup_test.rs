#![no_std]
#![no_main]

use core::{
    marker::PhantomData,
    sync::atomic::{compiler_fence, Ordering},
};

use embassy_executor::Spawner;
use esp32_wifi_hal_rs::{DMAResources, TxErrorBehaviour, TxParameters, WiFi, WiFiRate};
use esp_backtrace as _;
use esp_hal::{efuse::Efuse, timer::timg::TimerGroup};
use ieee80211::{
    common::{CapabilitiesInformation, SequenceControl},
    element_chain,
    elements::{
        tim::{StaticBitmap, TIMBitmap, TIMElement},
        DSSSParameterSetElement,
    },
    mac_parser::{MACAddress, BROADCAST},
    mgmt_frame::{body::BeaconBody, BeaconFrame, ManagementFrameHeader},
    scroll::Pwrite,
    ssid, supported_rates,
};

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_println::logger::init_logger_from_env();

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let dma_resources = mk_static!(DMAResources<1500, 10>, DMAResources::new());

    let wifi = WiFi::new(
        peripherals.WIFI,
        peripherals.RADIO_CLK,
        peripherals.ADC2,
        dma_resources,
    );
    let module_mac_address = MACAddress::new(Efuse::read_base_mac_address());
    /*
        wifi.set_filter(
            RxFilterBank::ReceiverAddress,
            RxFilterInterface::Zero,
            *module_mac_address,
            [0xff; 6],
        );
        wifi.set_filter_status(RxFilterBank::ReceiverAddress, RxFilterInterface::Zero, true);
    */
    let buf = mk_static!([u8; 1500], [0x00u8; 1500]);
    let mut seq_num = 0;
    loop {
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
                timestamp: 0,
                capabilities_info: CapabilitiesInformation::new().with_is_ess(true),
                elements: element_chain! {
                    ssid!("Test"),
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
        let written = buf.pwrite(frame, 0).unwrap();
        wifi.transmit(
            &mut buf[..written],
            &TxParameters {
                rate: if seq_num % 2 == 0 {
                    WiFiRate::PhyRate1ML
                } else {
                    WiFiRate::PhyRate2ML
                },
                interface_zero: false,
                interface_one: false,
                wait_for_ack: false,
                duration: 0,
                tx_error_behaviour: TxErrorBehaviour::Drop,
            },
        )
        .await
        .unwrap();
        seq_num += 1;
        compiler_fence(Ordering::SeqCst);
    }
}
