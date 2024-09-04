use core::{
    cell::RefCell,
    future::poll_fn,
    ops::Deref,
    sync::atomic::{AtomicUsize, Ordering},
    task::Poll,
};

use atomic_waker::AtomicWaker;
use critical_section::Mutex;
use esp_hal::{
    get_core,
    interrupt::{bind_interrupt, enable, map, CpuInterrupt, Priority},
    macros::ram,
    peripherals::{Interrupt, ADC2, LPWR, RADIO_CLK, WIFI},
    system::{RadioClockController, RadioPeripherals},
};
use esp_wifi_sys::include::{
    esp_phy_calibration_data_t, esp_phy_calibration_mode_t_PHY_RF_CAL_FULL, register_chipv7_phy,
};

use crate::{
    dma_list::{DMAList, DMAListItem},
    ffi::{
        chip_v7_set_chan_nomac, disable_wifi_agc, enable_wifi_agc, hal_init, wifi_set_rx_policy,
    },
    phy_init_data::PHY_INIT_DATA_DEFAULT,
    regs::{MAC_CTRL_REG, MAC_DMA_INT_CLEAR, MAC_DMA_INT_STATUS, MAC_RX_CTRL_REG},
};
use log::debug;

pub struct SignalQueue {
    waker: AtomicWaker,
    queued_signals: AtomicUsize,
}
impl SignalQueue {
    pub const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
            queued_signals: AtomicUsize::new(0),
        }
    }
    /// Increments the queue signals by one.
    pub fn put(&self) {
        self.queued_signals.fetch_add(1, Ordering::Relaxed);
        self.waker.wake();
    }
    pub async fn next(&self) {
        poll_fn(|cx| {
            let queued_signals = self.queued_signals.load(Ordering::Relaxed);
            if queued_signals == 0 {
                self.waker.register(cx.waker());
                Poll::Pending
            } else {
                self.queued_signals
                    .store(queued_signals - 1, Ordering::Relaxed);
                Poll::Ready(())
            }
        })
        .await
    }
}
static WIFI_RX_SIGNAL_QUEUE: SignalQueue = SignalQueue::new();

#[ram]
extern "C" fn interrupt_handler() {
    let cause = unsafe { MAC_DMA_INT_STATUS.read_volatile() };
    if cause == 0 {
        return;
    }
    if cause & 0x1000024 != 0 {
        WIFI_RX_SIGNAL_QUEUE.put();
    }
    unsafe { MAC_DMA_INT_CLEAR.write_volatile(cause) }
}
pub struct BorrowedBuffer<'a: 'b, 'b> {
    dma_list: &'a Mutex<RefCell<DMAList>>,
    dma_list_item: &'b mut DMAListItem,
}
impl Deref for BorrowedBuffer<'_, '_> {
    type Target = [u8];
    fn deref(&self) -> &Self::Target {
        self.dma_list_item.buffer()
    }
}
impl Drop for BorrowedBuffer<'_, '_> {
    fn drop(&mut self) {
        critical_section::with(|cs| self.dma_list.borrow_ref_mut(cs).recycle(self.dma_list_item));
    }
}
pub struct WiFi {
    radio_clock: RADIO_CLK,
    _adc2: ADC2,
    dma_list: Mutex<RefCell<DMAList>>,
}
impl WiFi {
    fn radio_peripheral_name(radio_peripheral: &RadioPeripherals) -> &'static str {
        match radio_peripheral {
            RadioPeripherals::Bt => "BT",
            RadioPeripherals::Phy => "PHY",
            RadioPeripherals::Wifi => "WiFi",
        }
    }
    fn enable_wifi_power_domain() {
        unsafe {
            let rtc_cntl = &*LPWR::ptr();
            debug!("Enabling wifi power domain.");
            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.wifi_force_pd().clear_bit());

            rtc_cntl
                .dig_iso()
                .modify(|_, w| w.wifi_force_iso().clear_bit());
        }
    }
    fn disable_wifi_power_domain() {
        unsafe {
            let rtc_cntl = &*LPWR::ptr();
            debug!("Disabling WiFi power domain.");
            rtc_cntl
                .dig_iso()
                .modify(|_, w| w.wifi_force_iso().set_bit());
            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.wifi_force_pd().set_bit());
        }
    }
    fn enable_clock(&mut self, radio_peripheral: RadioPeripherals) {
        debug!(
            "Enabling {} clock.",
            Self::radio_peripheral_name(&radio_peripheral)
        );
        self.radio_clock.enable(radio_peripheral);
    }
    fn disable_clock(&mut self, radio_peripheral: RadioPeripherals) {
        debug!(
            "Disabling {} clock.",
            Self::radio_peripheral_name(&radio_peripheral)
        );
        self.radio_clock.disable(radio_peripheral);
    }
    fn phy_enable(&mut self) {
        self.enable_clock(RadioPeripherals::Phy);
        let mut cal_data = [0u8; size_of::<esp_phy_calibration_data_t>()];
        let init_data = &PHY_INIT_DATA_DEFAULT;
        debug!("Enabling PHY.");
        unsafe {
            register_chipv7_phy(
                init_data,
                &mut cal_data as *mut _ as *mut esp_phy_calibration_data_t,
                esp_phy_calibration_mode_t_PHY_RF_CAL_FULL,
            );
        }
    }
    fn reset_mac(&mut self) {
        debug!("Reseting MAC.");
        self.radio_clock.reset_mac();
        unsafe {
            MAC_RX_CTRL_REG.write_volatile(MAC_RX_CTRL_REG.read_volatile() & 0x7fffffff);
        }
    }
    fn init_mac() {
        debug!("Initializing MAC.");
        unsafe {
            MAC_CTRL_REG.write_volatile(MAC_CTRL_REG.read_volatile() & 0xffffe800);
        }
    }
    fn deinit_mac() {
        debug!("Deinitializing MAC.");
        unsafe {
            MAC_CTRL_REG.write_volatile(MAC_CTRL_REG.read_volatile() | 0x17ff);
        }
        while unsafe { MAC_CTRL_REG.read_volatile() } & 0x2000 != 0 {}
    }
    pub fn change_channel(&self, channel_number: u8) {
        debug!("Changing channel to {channel_number}");
        Self::deinit_mac();
        unsafe {
            chip_v7_set_chan_nomac(channel_number, 0);
            disable_wifi_agc();
        }
        Self::init_mac();
        unsafe {
            enable_wifi_agc();
        }
    }
    fn set_isr() {
        debug!("Setting interrupt handler.");
        unsafe {
            map(
                get_core(),
                Interrupt::WIFI_MAC,
                CpuInterrupt::Interrupt0LevelPriority1,
            );
            bind_interrupt(Interrupt::WIFI_MAC, interrupt_handler);
        };
        enable(Interrupt::WIFI_MAC, Priority::Priority1).unwrap();
    }
    fn ic_enable() {
        debug!("ic_enable");
        unsafe {
            hal_init();
        }
        Self::set_isr();
    }
    fn ic_enable_rx() {
        debug!("Enabling RX.");
        unsafe {
            MAC_RX_CTRL_REG.write_volatile(MAC_RX_CTRL_REG.read_volatile() | 0x80000000);
        }
    }
    fn chip_enable() {
        debug!("chip_enable");
        unsafe {
            wifi_set_rx_policy(3);
        }
        Self::ic_enable_rx();
    }
    pub fn new(_wifi: WIFI, radio_clock: RADIO_CLK, _adc2: ADC2) -> Self {
        debug!("Initializing WiFi.");
        let mut temp = Self {
            radio_clock,
            _adc2,
            dma_list: Mutex::new(RefCell::new(DMAList::allocate(10))),
        };
        Self::enable_wifi_power_domain();
        temp.enable_clock(RadioPeripherals::Wifi);
        temp.phy_enable();
        temp.reset_mac();
        Self::init_mac();
        temp.change_channel(1);
        Self::ic_enable();
        Self::chip_enable();
        critical_section::with(|cs| temp.dma_list.borrow_ref_mut(cs).init());
        temp
    }
    pub async fn receive<'a>(&'a self) -> BorrowedBuffer<'a, '_> {
        let dma_list_item;

        // Sometimes the DMA list descriptors don't contain any data, even though the hardware indicated reception.
        // We loop until we get something.
        loop {
            WIFI_RX_SIGNAL_QUEUE.next().await;
            if let Some(current) =
                critical_section::with(|cs| self.dma_list.borrow_ref_mut(cs).take_first())
            {
                dma_list_item = current;
                break;
            }
            debug!("Received empty packet.");
        }

        BorrowedBuffer {
            dma_list: &self.dma_list,
            dma_list_item,
        }
    }
}
impl Drop for WiFi {
    fn drop(&mut self) {
        // Ensure, that the radio clocks and the power domain are disabled.
        self.disable_clock(RadioPeripherals::Wifi);
        self.disable_clock(RadioPeripherals::Phy);
        Self::disable_wifi_power_domain();
    }
}
