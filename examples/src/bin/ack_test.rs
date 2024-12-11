#![no_std]
#![no_main]
use core::{marker::PhantomData, mem::MaybeUninit};

use embassy_executor::Spawner;
use esp32_wifi_hal_rs::{
    DMAResources, RxFilterBank, RxFilterInterface, TxErrorBehaviour, WiFi, WiFiRate,
};
use esp_backtrace as _;
use esp_hal::{efuse::Efuse, timer::timg::TimerGroup};
use ieee80211::{
    common::{IEEE80211AuthenticationAlgorithmNumber, IEEE80211StatusCode},
    element_chain,
    mac_parser::MACAddress,
    mgmt_frame::{body::AuthenticationBody, AuthenticationFrame, ManagementFrameHeader},
    scroll::Pwrite,
};
use log::LevelFilter;

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
    let own_mac_address = MACAddress::new(Efuse::get_mac_address());
    let ap_address = MACAddress::new([0x00, 0x11, 0x22, 0x33, 0x44, 0x55]);
    wifi.set_filter_status(RxFilterBank::ReceiverAddress, RxFilterInterface::Zero, true);
    wifi.set_filter(
        RxFilterBank::ReceiverAddress,
        RxFilterInterface::Zero,
        own_mac_address.0,
        [0xff; 6],
    );
    wifi.set_filter_status(RxFilterBank::BSSID, RxFilterInterface::Zero, true);
    wifi.set_filter(
        RxFilterBank::BSSID,
        RxFilterInterface::Zero,
        ap_address.0,
        [0xff; 6],
    );
    let mut buf = [0x00u8; 300];
    let written = buf
        .pwrite(
            AuthenticationFrame {
                header: ManagementFrameHeader {
                    receiver_address: ap_address,
                    transmitter_address: own_mac_address,
                    bssid: ap_address,
                    ..Default::default()
                },
                body: AuthenticationBody {
                    status_code: IEEE80211StatusCode::Success,
                    authentication_algorithm_number:
                        IEEE80211AuthenticationAlgorithmNumber::OpenSystem,
                    authentication_transaction_sequence_number: 1,

                    elements: element_chain! {},
                    _phantom: PhantomData,
                },
            },
            0,
        )
        .unwrap();
    let slice = &buf[..written];
    loop {
        let _ = wifi
            .transmit(slice, WiFiRate::PhyRate9M, TxErrorBehaviour::Drop)
            .await;
    }
}
