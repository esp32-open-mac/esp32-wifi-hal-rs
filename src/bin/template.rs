#![no_std]
#![no_main]
use core::mem::MaybeUninit;

use embassy_executor::Spawner;
use esp32_wifi_hal_rs::WiFi;
use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;
use log::LevelFilter;

use esp_alloc as _;

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

    let _wifi = WiFi::new(peripherals.WIFI, peripherals.RADIO_CLK, peripherals.ADC2);
}
