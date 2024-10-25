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
use esp32::wifi::TX_SLOT_CONFIG;
use esp_hal::{
    get_core,
    interrupt::{bind_interrupt, enable, map, CpuInterrupt, Priority},
    macros::ram,
    peripherals::{Interrupt, ADC2, LPWR, RADIO_CLK, WIFI},
    system::{RadioClockController, RadioPeripherals},
};
use esp_wifi_sys::include::{
    esp_phy_calibration_data_t, esp_phy_calibration_mode_t_PHY_RF_CAL_FULL, register_chipv7_phy,
    wifi_pkt_rx_ctrl_t,
};
use ieee80211::common::HtSig;
use macro_bits::serializable_enum;

use crate::{
    dma_list::{DMAList, DMAListItem, RxDMAListItem, TxDMAListItem},
    ffi::{
        chip_v7_set_chan_nomac, disable_wifi_agc, enable_wifi_agc, hal_init, tx_pwctrl_background,
    },
    phy_init_data::PHY_INIT_DATA_DEFAULT,
};
use log::{debug, error, trace, warn};

serializable_enum! {
    #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
    /// The rate used by the PHY.
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
        PhyRate9M => 0x0f,
        PhyRateMCS0LGI => 0x10,
        PhyRateMCS1LGI => 0x11,
        PhyRateMCS2LGI => 0x12,
        PhyRateMCS3LGI => 0x13,
        PhyRateMCS4LGI => 0x14,
        PhyRateMCS5LGI => 0x15,
        PhyRateMCS6LGI => 0x16,
        PhyRateMCS7LGI => 0x17,
        PhyRateMCS0SGI => 0x18,
        PhyRateMCS1SGI => 0x19,
        PhyRateMCS2SGI => 0x1a,
        PhyRateMCS3SGI => 0x1b,
        PhyRateMCS4SGI => 0x1c,
        PhyRateMCS5SGI => 0x1d,
        PhyRateMCS6SGI => 0x1e,
        PhyRateMCS7SGI => 0x1f
    }
}
impl WiFiRate {
    pub const fn is_ht(&self) -> bool {
        self.into_bits() >= 0x10
    }
    pub const fn is_short_gi(&self) -> bool {
        self.into_bits() >= 0x18
    }
}

struct SignalQueue {
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
const TX_COLLISION: u8 = 3;

/// This is used to wait for a slot to become available.
static WIFI_TX_SLOT_QUEUE: SlotManager = SlotManager::new();
struct BorrowedSlot {
    slot: usize,
}
impl Deref for BorrowedSlot {
    type Target = usize;
    fn deref(&self) -> &Self::Target {
        &self.slot
    }
}
impl Drop for BorrowedSlot {
    fn drop(&mut self) {
        let _ = WIFI_TX_SLOT_QUEUE.slots.try_send(self.slot);
    }
}
struct SlotManager {
    slots: Channel<CriticalSectionRawMutex, usize, 5>,
}
impl SlotManager {
    pub const fn new() -> Self {
        Self {
            slots: Channel::new(),
        }
    }
    pub fn init(&self, slots: impl IntoIterator<Item = usize>) {
        assert!(self.slots.is_empty());
        slots.into_iter().for_each(|slot| {
            let _ = self.slots.try_send(slot);
        });
    }
    pub async fn wait_for_slot(&self) -> BorrowedSlot {
        BorrowedSlot {
            slot: self.slots.receive().await,
        }
    }
}

static WIFI_RX_SIGNAL_QUEUE: SignalQueue = SignalQueue::new();
#[allow(clippy::declare_interior_mutable_const)]
const SLOT: (AtomicWaker, AtomicU8) = (AtomicWaker::new(), AtomicU8::new(TX_IN_PROGRESS_OR_FREE));
/// These are for knowing, when transmission has finished.
static WIFI_TX_SLOTS: [(AtomicWaker, AtomicU8); 5] = [SLOT; 5];
static FRAMES_SINCE_LAST_TXPWR_CTRL: AtomicU8 = AtomicU8::new(0);

fn process_tx_complete(slot: usize) {
    if FRAMES_SINCE_LAST_TXPWR_CTRL.fetch_add(1, Ordering::Relaxed) == 4 {
        unsafe { tx_pwctrl_background(1, 0) };
        FRAMES_SINCE_LAST_TXPWR_CTRL.store(0, Ordering::Relaxed);
    }
    unsafe { WIFI::steal() }
        .tx_complete_clear()
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << slot)) });
    if slot < 5 {
        WIFI_TX_SLOTS[slot].1.store(TX_DONE, Ordering::Relaxed);
        WIFI_TX_SLOTS[slot].0.wake();
    }
}
fn process_tx_timeout(slot: usize) {
    unsafe { WIFI::steal() }
        .tx_error_clear()
        .modify(|r, w| unsafe { w.slot_timeout().bits(r.slot_timeout().bits() | (1 << slot)) });
    if slot < 5 {
        WiFi::set_tx_slot_invalid(unsafe { WIFI::steal() }.tx_slot_config(4 - slot));
        WIFI_TX_SLOTS[slot].1.store(TX_TIMEOUT, Ordering::Relaxed);
        WIFI_TX_SLOTS[slot].0.wake();
    }
}
fn process_collision(slot: usize) {
    unsafe { WIFI::steal() }
        .tx_error_clear()
        .modify(|r, w| unsafe {
            w.slot_collision()
                .bits(r.slot_collision().bits() | (1 << slot))
        });
    if slot < 5 {
        WiFi::set_tx_slot_invalid(unsafe { WIFI::steal() }.tx_slot_config(4 - slot));
        WIFI_TX_SLOTS[slot].1.store(TX_COLLISION, Ordering::Relaxed);
        WIFI_TX_SLOTS[slot].0.wake();
    }
}

#[ram]
extern "C" fn interrupt_handler() {
    // We don't want to have to steal this all the time.
    let wifi = unsafe { WIFI::steal() };

    let cause = wifi.wifi_int_status().read().bits();
    if cause == 0 {
        return;
    }
    wifi.wifi_int_clear().write(|w| unsafe { w.bits(cause) });
    if cause & 0x1000024 != 0 {
        WIFI_RX_SIGNAL_QUEUE.put();
    } else if cause & 0x80 != 0 {
        //let mut txq_complete_status = wifi.txq_complete_status().read().bits();
        let mut txq_complete_status = unsafe { WIFI::steal() }.tx_complete_status().read().bits();
        while txq_complete_status != 0 {
            let slot = txq_complete_status.trailing_zeros();
            process_tx_complete(slot as usize);
            // We mask away, the bit for our slot.
            txq_complete_status &= !(1 << slot);
        }
    } else if cause & 0x80000 != 0 {
        // Timeout
        let mut tx_error_status = unsafe { WIFI::steal() }
            .tx_error_status()
            .read()
            .slot_timeout()
            .bits();
        while tx_error_status != 0 {
            let slot = tx_error_status.trailing_zeros();
            process_tx_timeout(slot as usize);
            // We mask away, the bit for our slot.
            tx_error_status &= !(1 << slot);
        }
    } else if cause & 0x100 != 0 {
        // Timeout
        let mut tx_error_status = unsafe { WIFI::steal() }
            .tx_error_status()
            .read()
            .slot_collision()
            .bits();
        while tx_error_status != 0 {
            let slot = tx_error_status.trailing_zeros();
            process_collision(slot as usize);
            // We mask away, the bit for our slot.
            tx_error_status &= !(1 << slot);
        }
    }
}

/* /// What should be done, if a timeout occurs, while transmitting.
pub enum TxTimeoutBehaviour {
    /// Retry transmitting the ppdu over and over again.
    Retry,
    /// Drop the ppdu.
    Drop,
} */

/// A buffer borrowed from the DMA list.
pub struct BorrowedBuffer<'a: 'b, 'b> {
    dma_list: &'a Mutex<RefCell<DMAList>>,
    dma_list_item: &'b mut RxDMAListItem,
}
impl<'a: 'b, 'b> BorrowedBuffer<'a, 'b> {
    /// Returns the actual MPDU from the buffer excluding the prepended [wifi_pkt_rx_ctrl_t].
    pub fn mpdu_buffer(&'a self) -> &'b [u8] {
        &self[size_of::<wifi_pkt_rx_ctrl_t>()..]
    }
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
serializable_enum! {
    /// The bank of the rx filter.
    #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
    pub enum RxFilterBank : u8 {
        BSSID => 0,
        ReceiverAddress => 1
    }
}
serializable_enum! {
    /// The interface of the rx filter.
    #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
    pub enum RxFilterInterface : u8 {
        Zero => 0,
        One => 1
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum WiFiError {
    InvalidChannel,
}
pub type WiFiResult<T> = Result<T, WiFiError>;
pub struct WiFi {
    radio_clock: RADIO_CLK,
    wifi: WIFI,
    dma_list: Mutex<RefCell<DMAList>>,
    current_channel: u8,
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
            trace!("Enabling wifi power domain.");
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
            trace!("Disabling WiFi power domain.");
            rtc_cntl
                .dig_iso()
                .modify(|_, w| w.wifi_force_iso().set_bit());
            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.wifi_force_pd().set_bit());
        }
    }
    fn enable_clock(&mut self, radio_peripheral: RadioPeripherals) {
        trace!(
            "Enabling {} clock.",
            Self::radio_peripheral_name(&radio_peripheral)
        );
        self.radio_clock.enable(radio_peripheral);
    }
    fn disable_clock(&mut self, radio_peripheral: RadioPeripherals) {
        trace!(
            "Disabling {} clock.",
            Self::radio_peripheral_name(&radio_peripheral)
        );
        self.radio_clock.disable(radio_peripheral);
    }
    fn phy_enable(&mut self) {
        self.enable_clock(RadioPeripherals::Phy);
        let mut cal_data = [0u8; size_of::<esp_phy_calibration_data_t>()];
        let init_data = &PHY_INIT_DATA_DEFAULT;
        trace!("Enabling PHY.");
        unsafe {
            register_chipv7_phy(
                init_data,
                &mut cal_data as *mut _ as *mut esp_phy_calibration_data_t,
                esp_phy_calibration_mode_t_PHY_RF_CAL_FULL,
            );
        }
    }
    fn reset_mac(&mut self) {
        trace!("Reseting MAC.");
        self.radio_clock.reset_mac();
        self.wifi
            .ctrl()
            .modify(|r, w| unsafe { w.bits(r.bits() & 0x7fffffff) });
    }
    fn init_mac(&self) {
        trace!("Initializing MAC.");
        self.wifi
            .ctrl()
            .modify(|r, w| unsafe { w.bits(r.bits() & 0xffffe800) });
    }
    fn deinit_mac(&self) {
        trace!("Deinitializing MAC.");
        self.wifi.ctrl().modify(|r, w| unsafe {
            w.bits(r.bits() | 0x17ff);
            while r.bits() & 0x2000 != 0 {}
            w
        });
    }
    /// Set the channel on which to operate.
    ///
    /// NOTE:
    /// This uses the proprietary blob.
    pub fn set_channel(&mut self, channel_number: u8) -> WiFiResult<()> {
        if !(1..=14).contains(&channel_number) {
            return Err(WiFiError::InvalidChannel);
        }
        trace!("Changing channel to {channel_number}");
        self.deinit_mac();
        unsafe {
            chip_v7_set_chan_nomac(channel_number, 0);
            disable_wifi_agc();
        }
        self.init_mac();
        unsafe {
            enable_wifi_agc();
        }
        self.current_channel = channel_number;
        Ok(())
    }
    /// Returns the current channel.
    pub fn get_channel(&self) -> u8 {
        self.current_channel
    }
    fn set_isr() {
        trace!("Setting interrupt handler.");
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
        trace!("ic_enable");
        unsafe {
            hal_init();
        }
        Self::set_isr();
    }
    fn ic_enable_rx(&mut self) {
        trace!("Enabling RX.");
        self.wifi.rx_ctrl().write(|w| w.rx_enable().bit(true));
    }
    fn chip_enable(&mut self) {
        trace!("chip_enable");
        self.ic_enable_rx();
    }
    /// Initialize the WiFi peripheral.
    pub fn new(wifi: WIFI, radio_clock: RADIO_CLK, _adc2: ADC2) -> Self {
        trace!("Initializing WiFi.");
        let mut temp = Self {
            radio_clock,
            wifi,
            dma_list: Mutex::new(RefCell::new(DMAList::allocate(10))),
            current_channel: 1,
        };
        Self::enable_wifi_power_domain();
        temp.enable_clock(RadioPeripherals::Wifi);
        temp.phy_enable();
        temp.reset_mac();
        temp.init_mac();
        temp.set_channel(1).unwrap();
        Self::ic_enable();
        temp.chip_enable();
        // Initialize DMA list.
        critical_section::with(|cs| temp.dma_list.borrow_ref_mut(cs).init());
        WIFI_TX_SLOT_QUEUE.init(0..5);
        debug!("WiFi MAC init complete.");
        temp
    }
    /// Receive a frame.
    pub async fn receive<'a: 'b, 'b>(&'a self) -> BorrowedBuffer<'a, 'b> {
        let dma_list_item;

        // Sometimes the DMA list descriptors don't contain any data, even though the hardware indicated reception.
        // We loop until we get something.
        loop {
            WIFI_RX_SIGNAL_QUEUE.next().await;
            if let Some(current) =
                critical_section::with(|cs| self.dma_list.borrow_ref_mut(cs).take_first())
            {
                trace!("Received packet. len: {}", current.buffer().len());
                dma_list_item = current;
                break;
            }
            trace!("Received empty packet.");
        }

        BorrowedBuffer {
            dma_list: &self.dma_list,
            dma_list_item,
        }
    }
    fn enable_tx_slot(tx_slot_config: &TX_SLOT_CONFIG) {
        tx_slot_config
            .plcp0()
            .modify(|r, w| unsafe { w.bits(r.bits() | 0xc0000000) });
    }
    pub(crate) fn set_tx_slot_invalid(tx_slot_config: &TX_SLOT_CONFIG) {
        tx_slot_config
            .plcp0()
            .modify(|r, w| unsafe { w.bits(r.bits() | 0xb0000000) });
    }
    async fn transmit_internal(
        &self,
        dma_list_item: Pin<&TxDMAListItem>,
        rate: WiFiRate,
        slot: usize,
    ) -> bool {
        let length = dma_list_item.buffer().len();

        let tx_slot_config = self.wifi.tx_slot_config(4 - slot);
        tx_slot_config
            .config()
            .modify(|r, w| unsafe { w.bits(r.bits() | 0xa) });
        tx_slot_config.plcp0().write(|w| unsafe {
            w.dma_addr()
                .bits(dma_list_item.get_ref() as *const _ as u32)
        });
        tx_slot_config
            .plcp0()
            .modify(|r, w| unsafe { w.bits(r.bits() | 0x00600000) });

        let tx_slot_parameters = self.wifi.tx_slot_parameters(4 - slot);
        tx_slot_parameters.plcp1().write(|w| unsafe {
            w.len()
                .bits(length as u16)
                .is_80211_n()
                .bit(rate.is_ht())
                .rate()
                .bits(rate.into_bits())
        });
        tx_slot_parameters
            .plcp2()
            .write(|w| unsafe { w.bits(0x00000020) });
        tx_slot_parameters
            .duration()
            .write(|w| unsafe { w.bits(0x0) });
        if rate.is_ht() {
            tx_slot_parameters.ht_sig().write(|w| unsafe {
                w.bits(
                    HtSig::new()
                        .with_ht_length(length as u16)
                        .with_mcs(rate.into_bits() & 0b111)
                        .with_smoothing_recommended(true)
                        .with_not_sounding(true)
                        .with_short_gi(rate.is_short_gi())
                        .into_bits(),
                )
            });
            tx_slot_parameters
                .ht_unknown()
                .write(|w| unsafe { w.length().bits(length as u32) });
        }
        tx_slot_config
            .config()
            .modify(|r, w| unsafe { w.bits(r.bits() | 0x02000000) });
        Self::enable_tx_slot(tx_slot_config);
        struct CancelOnDrop<'a> {
            tx_slot_config: &'a TX_SLOT_CONFIG,
            slot: usize,
        }
        impl CancelOnDrop<'_> {
            async fn wait_for_tx_complete(&self) -> bool {
                let slot = self.slot;
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
                    TX_COLLISION => {
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
        }
        impl Drop for CancelOnDrop<'_> {
            fn drop(&mut self) {
                WiFi::set_tx_slot_invalid(self.tx_slot_config);
            }
        }
        CancelOnDrop {
            tx_slot_config,
            slot,
        }
        .wait_for_tx_complete()
        .await
    }
    /// Transmit a frame.
    ///
    /// NOTE: An FCS must be attached.
    pub async fn transmit(&self, buffer: &[u8], rate: WiFiRate) -> bool {
        let slot = WIFI_TX_SLOT_QUEUE.wait_for_slot().await;
        trace!("Acquired slot {}.", *slot);

        // We initialize the DMA list item.
        let mut dma_list_item = DMAListItem::new_for_tx(buffer);

        // And then pin it, before passing it to hardware.
        let dma_list_item = pin!(dma_list_item);

        let tx_success = self
            .transmit_internal(dma_list_item.into_ref(), rate, *slot)
            .await;
        if tx_success {
            trace!("Finished transmission. Slot {} is now free again.", *slot);
        } else {
            warn!("Timeout occured for slot {}.", *slot);
        }
        tx_success
    }
    pub fn set_filter_status(
        &mut self,
        bank: RxFilterBank,
        interface: RxFilterInterface,
        enabled: bool,
    ) {
        self.wifi
            .filter_bank(bank.into_bits() as usize)
            .mask_high(interface.into_bits() as usize)
            .write(|w| w.enabled().bit(enabled));
    }
    pub fn set_filter(
        &mut self,
        bank: RxFilterBank,
        interface: RxFilterInterface,
        address: [u8; 6],
        mask: [u8; 6],
    ) {
        let bank = self.wifi.filter_bank(bank.into_bits() as usize);
        let interface = interface.into_bits() as usize;
        bank.addr_low(interface)
            .write(|w| unsafe { w.bits(u32::from_le_bytes(address[..4].try_into().unwrap())) });
        bank.addr_high(interface).write(|w| unsafe {
            w.addr()
                .bits(u16::from_le_bytes(address[4..6].try_into().unwrap()))
        });
        bank.mask_low(interface)
            .write(|w| unsafe { w.bits(u32::from_le_bytes(mask[..4].try_into().unwrap())) });
        bank.mask_high(interface).write(|w| unsafe {
            w.mask()
                .bits(u16::from_le_bytes(mask[4..6].try_into().unwrap()))
        });
    }
    pub fn set_scanning_mode(&self, interface: RxFilterInterface, enable: bool) {
        self.wifi
            .unknown_rx_policy(interface.into_bits() as usize)
            .modify(|r, w| unsafe {
                w.bits(if enable {
                    r.bits() | 0x110
                } else {
                    r.bits() & (!0x110)
                })
            });
    }
}
macro_rules! generate_stat_accessors {
    ($(
        $stat_name: ident
    ),*) => {
        impl WiFi {
            $(
                pub fn $stat_name(&self) -> u32 {
                    self.wifi.$stat_name().read().bits()
                }
            )*
        }
    };
}
generate_stat_accessors![hw_stat_panic];
impl Drop for WiFi {
    fn drop(&mut self) {
        // Ensure, that the radio clocks and the power domain are disabled.
        self.disable_clock(RadioPeripherals::Wifi);
        self.disable_clock(RadioPeripherals::Phy);
        Self::disable_wifi_power_domain();
    }
}
