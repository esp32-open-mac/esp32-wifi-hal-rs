use core::{
    cell::RefCell,
    future::poll_fn,
    ops::Deref,
    pin::{pin, Pin},
    sync::atomic::{AtomicU8, AtomicUsize, Ordering},
    task::Poll,
};

use atomic_waker::AtomicWaker;
use critical_section::Mutex;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
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
use macro_bits::serializable_enum;

use crate::{
    dma_list::{DMAList, DMAListItem},
    ffi::{
        chip_v7_set_chan_nomac, disable_wifi_agc, enable_wifi_agc, hal_init, tx_pwctrl_background,
        wifi_set_rx_policy,
    },
    phy_init_data::PHY_INIT_DATA_DEFAULT,
    regs::{
        duration, plcp0, plcp1, plcp2, tx_config, MAC_CTRL_REG, MAC_DMA_INT_CLEAR,
        MAC_DMA_INT_STATUS, MAC_RX_CTRL_REG, MAC_TXQ_COMPLETE_CLEAR, MAC_TXQ_COMPLETE_STATUS,
        MAC_TXQ_ERROR_CLEAR, MAC_TXQ_ERROR_STATUS,
    },
};
use log::{debug, error, trace, warn};

serializable_enum! {
    pub enum WiFiRate: u8 {
        PhyRate1ML => 0x00,
        PhyRate2ML => 0x01,
        PhyRate5ML => 0x02,
        PhyRate11ML => 0x03,
        PhyRate2MS => 0x05,
        PhyRate5MS => 0x06,
        PhyRate11MS => 0x07,
        PhyRate48M => 0x08,
        PhyRate24M => 0x09,
        PhyRate12M => 0x0a,
        PhyRate6M => 0x0b,
        PhyRate54M => 0x0c,
        PhyRate36M => 0x0d,
        PhyRate18M => 0x0e,
        PhyRate9M => 0x0f
    }
}

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
// Status codes for TX.
const TX_IN_PROGRESS_OR_FREE: u8 = 0;
const TX_DONE: u8 = 1;
const TX_TIMEOUT: u8 = 2;

static WIFI_RX_SIGNAL_QUEUE: SignalQueue = SignalQueue::new();
#[allow(clippy::declare_interior_mutable_const)]
const SLOT: (AtomicWaker, AtomicU8) = (AtomicWaker::new(), AtomicU8::new(TX_IN_PROGRESS_OR_FREE));
/// These are for knowing, when transmission has finished.
static WIFI_TX_SLOTS: [(AtomicWaker, AtomicU8); 5] = [SLOT; 5];
/// This is used to wait for a slot to become available.
static WIFI_TX_SLOT_QUEUE: Channel<CriticalSectionRawMutex, usize, 5> = Channel::new();
static FRAMES_SINCE_LAST_TXPWR_CTRL: AtomicU8 = AtomicU8::new(0);

fn process_tx_complete(slot: usize) {
    unsafe { MAC_TXQ_COMPLETE_CLEAR.write_volatile(1 << slot) };
    if slot < 5 {
        WIFI_TX_SLOTS[slot].1.store(TX_DONE, Ordering::Relaxed);
        WIFI_TX_SLOTS[slot].0.wake();
    }
}
fn process_tx_timeout(slot: usize) {
    unsafe { MAC_TXQ_ERROR_CLEAR.write_volatile(1 << slot) };
    if FRAMES_SINCE_LAST_TXPWR_CTRL.fetch_add(1, Ordering::Relaxed) == 4 {
        unsafe { tx_pwctrl_background(1, 0) };
        FRAMES_SINCE_LAST_TXPWR_CTRL.store(0, Ordering::Relaxed);
    }
    if slot < 5 {
        set_txq_invalid(slot);
        WIFI_TX_SLOTS[slot].1.store(TX_TIMEOUT, Ordering::Relaxed);
        WIFI_TX_SLOTS[slot].0.wake();
    }
}

#[ram]
extern "C" fn interrupt_handler() {
    let cause = unsafe { MAC_DMA_INT_STATUS.read_volatile() };
    if cause == 0 {
        return;
    }
    unsafe { MAC_DMA_INT_CLEAR.write_volatile(cause) }
    if cause & 0x1000024 != 0 {
        WIFI_RX_SIGNAL_QUEUE.put();
    } else if cause & 0x80 != 0 {
        let mut txq_complete_status = unsafe { MAC_TXQ_COMPLETE_STATUS.read_volatile() };
        while txq_complete_status != 0 {
            let slot = 31 - txq_complete_status.leading_zeros();
            process_tx_complete(slot as usize);
            // We mask away, the bit for our slot.
            txq_complete_status &= !(1 << slot);
        }
    } else if cause & 0x80000 != 0 {
        let mut txq_error_status = unsafe { MAC_TXQ_ERROR_STATUS.read_volatile() };
        while txq_error_status != 0 {
            let slot = 31 - txq_error_status.leading_zeros();
            process_tx_timeout(slot as usize);
            // We mask away, the bit for our slot.
            txq_error_status &= !(1 << slot);
        }
    } else {
        trace!("Got unknown interrupt: {cause:x}.");
    }
}
fn enable_tx(slot: usize) {
    let plcp0_ptr = plcp0(slot);
    unsafe {
        plcp0_ptr.write_volatile(plcp0_ptr.read_volatile() | 0xc0000000);
    }
}
fn set_txq_invalid(slot: usize) {
    let plcp0_ptr = plcp0(slot);
    unsafe {
        plcp0_ptr.write_volatile(plcp0_ptr.read_volatile() & 0xb0000000);
    }
}
async fn transmit_internal(dma_list_item: Pin<&DMAListItem>, rate: WiFiRate, slot: usize) -> bool {
    let tx_config_ptr = tx_config(slot);
    let plcp0_ptr = plcp0(slot);
    let plcp1_ptr = plcp1(slot);
    let plcp2_ptr = plcp2(slot);
    let duration_ptr = duration(slot);
    unsafe {
        tx_config_ptr.write_volatile(tx_config_ptr.read_volatile() | 0xa);
        plcp0_ptr.write_volatile(dma_list_item.get_ref() as *const _ as u32 & 0xfffff | 0x00600000);
        plcp1_ptr.write_volatile(
            0x10000000
                | dma_list_item.buffer().len() as u32 & 0xfff
                | ((rate.into_bits() as u32 & 0x1f) << 12),
        );
        plcp2_ptr.write_volatile(0x00000020);
        duration_ptr.write_volatile(0);
        tx_config_ptr.write_volatile(tx_config_ptr.read_volatile() | 0x02000000);
        tx_config_ptr.write_volatile(tx_config_ptr.read_volatile() | 0x00003000);
    }
    enable_tx(slot);

    // Wait for the hardware to confirm transmission.
    poll_fn(|cx| match WIFI_TX_SLOTS[slot].1.load(Ordering::Relaxed) {
        TX_DONE => {
            WIFI_TX_SLOTS[slot]
                .1
                .store(TX_IN_PROGRESS_OR_FREE, Ordering::Relaxed);
            Poll::Ready(true)
        }
        TX_TIMEOUT => {
            WIFI_TX_SLOTS[slot]
                .1
                .store(TX_IN_PROGRESS_OR_FREE, Ordering::Relaxed);
            Poll::Ready(false)
        }
        TX_IN_PROGRESS_OR_FREE => {
            WIFI_TX_SLOTS[slot].0.register(cx.waker());
            Poll::Pending
        }
        status => {
            error!("TX slot status was set to invalid value: {status}");
            Poll::Ready(false)
        }
    })
    .await
}

/* /// What should be done, if a timeout occurs, while transmitting.
pub enum TxTimeoutBehaviour {
    /// Retry transmitting the ppdu over and over again.
    Retry,
    /// Drop the ppdu.
    Drop,
} */

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
        // Initialize DMA list.
        critical_section::with(|cs| temp.dma_list.borrow_ref_mut(cs).init());
        // Initialize TX slot queue
        (0..1).for_each(|slot| WIFI_TX_SLOT_QUEUE.try_send(slot).unwrap());
        temp
    }
    pub async fn receive(&self) -> BorrowedBuffer<'_, '_> {
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
    pub async fn transmit(&self, buffer: &[u8], rate: WiFiRate) -> bool {
        let slot = WIFI_TX_SLOT_QUEUE.receive().await;
        debug!("Acquired slot {slot}.");

        // We initialize the DMA list item.
        let mut dma_list_item = DMAListItem::new(buffer, None);
        dma_list_item.set_size(buffer.len() + 32);
        dma_list_item.set_has_data(true);

        // And then pin it, before passing it to hardware.
        let dma_list_item = pin!(dma_list_item);

        let tx_success = transmit_internal(dma_list_item.into_ref(), rate, slot).await;

        // Put the slot back into the availability queue.
        WIFI_TX_SLOT_QUEUE.send(slot).await;
        if tx_success {
            debug!("Finished transmission. Slot {slot} is now free again.");
        } else {
            warn!("Timeout occured for slot {slot}.");
        }
        tx_success
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
