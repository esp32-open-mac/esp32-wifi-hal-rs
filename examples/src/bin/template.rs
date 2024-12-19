#![no_std]
#![no_main]

use embassy_executor::Spawner;
use esp32_wifi_hal_rs::{DMAResources, WiFi};
use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;
use log::LevelFilter;

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
    esp_println::logger::init_logger(LevelFilter::Info);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let dma_resources = mk_static!(DMAResources<1500, 10>, DMAResources::new());

    let _wifi = WiFi::new(
        peripherals.WIFI,
        peripherals.RADIO_CLK,
        peripherals.ADC2,
        dma_resources,
    );
}
