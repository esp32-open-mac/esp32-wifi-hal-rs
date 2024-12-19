use core::{
    cell::RefCell,
    future::poll_fn,
    mem,
    ops::{Deref, Range},
    pin::{pin, Pin},
    sync::atomic::{AtomicU8, AtomicUsize, Ordering},
    task::Poll,
};

use atomic_waker::AtomicWaker;
use critical_section::Mutex;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, signal::Signal,
};
use embassy_time::Instant;
use esp32::wifi::TX_SLOT_CONFIG;
use esp_hal::{
    interrupt::{bind_interrupt, enable, map, CpuInterrupt, Priority},
    macros::ram,
    peripherals::{Interrupt, ADC2, LPWR, RADIO_CLK, WIFI},
    system::{RadioClockController, RadioPeripherals},
    Cpu,
};
use esp_wifi_sys::include::{
    esp_phy_calibration_data_t, esp_phy_calibration_mode_t_PHY_RF_CAL_FULL, register_chipv7_phy,
    wifi_pkt_rx_ctrl_t,
};
use macro_bits::{bit, check_bit, serializable_enum};

use crate::{
    dma_list::{DMAList, DMAListItem, RxDMAListItem, TxDMAListItem},
    ffi::{
        chip_v7_set_chan_nomac, disable_wifi_agc, enable_wifi_agc, hal_init, tx_pwctrl_background,
    },
    phy_init_data::PHY_INIT_DATA_DEFAULT,
    DMAResources,
};

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
    /// Check if the rate is using the HT PHY.
    pub const fn is_ht(&self) -> bool {
        self.into_bits() >= 0x10
    }
    /// Check if the rate uses a short guard interval.
    pub const fn is_short_gi(&self) -> bool {
        self.into_bits() >= 0x18
    }
}

/// A synchronization primitive, which allows queueing a number signals, to be awaited.
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

/// This is used to wait for a slot to become available.
static WIFI_TX_SLOT_QUEUE: SlotManager = SlotManager::new();

/// This is a slot borrowed from the slot queue.
///
/// It is used, to make sure that access to a slot is exclusive.
/// It will return the slot back into the slot queue once dropped.
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
        // We can ignore the result here, because we know that this slot was taken from the queue,
        // and therefore the queue must have space for it.
        let _ = WIFI_TX_SLOT_QUEUE.slots.try_send(self.slot);
    }
}
/// This keeps track of all the TX slots available, by using a queue of slot numbers in the
/// background, which makes it possible to await a slot becoming free.
struct SlotManager {
    slots: Channel<CriticalSectionRawMutex, usize, 5>,
}
impl SlotManager {
    /// Create a new slot manager.
    pub const fn new() -> Self {
        Self {
            slots: Channel::new(),
        }
    }
    /// Fill the slot queue, with the given range of slot numbers.
    ///
    /// Making the range of slots to use dynamic here makes sense, because we're unsure about the
    /// nature of slot four, and as such sometimes disable it for certain experiments.
    pub fn init(&self, slots: Range<usize>) {
        slots.into_iter().for_each(|slot| {
            let _ = self.slots.try_send(slot);
        });
    }
    /// Asynchronously wait for a new slot to become available.
    pub async fn wait_for_slot(&self) -> BorrowedSlot {
        BorrowedSlot {
            slot: self.slots.receive().await,
        }
    }
}

pub enum TxSlotStatus {
    Done,
    Timeout,
    Collision,
}
static WIFI_RX_SIGNAL_QUEUE: SignalQueue = SignalQueue::new();

#[allow(clippy::declare_interior_mutable_const)]
const EMPTY_SLOT: Signal<CriticalSectionRawMutex, TxSlotStatus> = Signal::new();
/// These are for knowing, when transmission has finished.
static WIFI_TX_SLOTS: [Signal<CriticalSectionRawMutex, TxSlotStatus>; 5] = [EMPTY_SLOT; 5];

// We run tx_pwctrl_background every four transmissions.
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
        WIFI_TX_SLOTS[slot].signal(TxSlotStatus::Done);
    }
}
fn process_tx_timeout(slot: usize) {
    unsafe { WIFI::steal() }
        .tx_error_clear()
        .modify(|r, w| unsafe { w.slot_timeout().bits(r.slot_timeout().bits() | (1 << slot)) });
    if slot < 5 {
        WiFi::set_tx_slot_invalid(unsafe { WIFI::steal() }.tx_slot_config(4 - slot));
        WIFI_TX_SLOTS[slot].signal(TxSlotStatus::Timeout);
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
        WIFI_TX_SLOTS[slot].signal(TxSlotStatus::Collision);
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

/// What should be done, if an error occurs, while transmitting.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum TxErrorBehaviour {
    /// Retry as many times, as specified.
    RetryUntil(usize),
    /// Drop the MPDU.
    #[default]
    Drop,
}

/// A buffer borrowed from the DMA list.
pub struct BorrowedBuffer<'res, 'a> {
    dma_list: &'a Mutex<RefCell<DMAList<'res>>>,
    dma_list_item: &'a mut RxDMAListItem,
}
impl BorrowedBuffer<'_, '_> {
    /// Returns the complete buffer returned by the hardware.
    ///
    /// This includes the header added by the hardware.
    pub fn raw_buffer(&self) -> &[u8] {
        self.dma_list_item.buffer()
    }
    /// Returns the actual MPDU from the buffer excluding the prepended [wifi_pkt_rx_ctrl_t].
    pub fn mpdu_buffer(&self) -> &[u8] {
        &self.raw_buffer()[size_of::<wifi_pkt_rx_ctrl_t>()..]
    }
    /// Returns the header attached by the hardware.
    pub fn header_buffer(&self) -> &[u8] {
        &self.raw_buffer()[0..size_of::<wifi_pkt_rx_ctrl_t>()]
    }
    /// The Received Signal Strength Indicator (RSSI).
    pub fn rssi(&self) -> i8 {
        self.header_buffer()[0] as i8 - 96
    }
    /// Is the frame intended for interface zero.
    pub fn interface_zero(&self) -> bool {
        check_bit!(self.header_buffer()[3], bit!(4))
    }
    /// Is the frame intended for interface one.
    pub fn interface_one(&self) -> bool {
        check_bit!(self.header_buffer()[3], bit!(5))
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
    TxTimeout,
    TxCollision,
}
pub type WiFiResult<T> = Result<T, WiFiError>;
pub struct WiFi<'res> {
    radio_clock: RADIO_CLK,
    wifi: WIFI,
    dma_list: Mutex<RefCell<DMAList<'res>>>,
    current_channel: AtomicU8,
}
impl<'res> WiFi<'res> {
    /// Returns the name of a radio peripheral.
    fn radio_peripheral_name(radio_peripheral: &RadioPeripherals) -> &'static str {
        match radio_peripheral {
            RadioPeripherals::Bt => "BT",
            RadioPeripherals::Phy => "PHY",
            RadioPeripherals::Wifi => "WiFi",
        }
    }
    /// Enable the Wi-Fi power domain.
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
    /// Disable the Wi-Fi power domain.
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
    /// Enable the specified clock.
    fn enable_clock(radio_clock: &mut RADIO_CLK, radio_peripheral: RadioPeripherals) {
        trace!(
            "Enabling {} clock.",
            Self::radio_peripheral_name(&radio_peripheral)
        );
        radio_clock.enable(radio_peripheral);
    }
    /// Disable the specified clock.
    fn disable_clock(radio_clock: &mut RADIO_CLK, radio_peripheral: RadioPeripherals) {
        trace!(
            "Disabling {} clock.",
            Self::radio_peripheral_name(&radio_peripheral)
        );
        radio_clock.disable(radio_peripheral);
    }
    /// Enable the PHY.
    fn phy_enable(radio_clock: &mut RADIO_CLK) {
        Self::enable_clock(radio_clock, RadioPeripherals::Phy);
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
    /// Reset the MAC.
    fn reset_mac(radio_clock: &mut RADIO_CLK, wifi: &WIFI) {
        trace!("Reseting MAC.");
        radio_clock.reset_mac();
        wifi.ctrl()
            .modify(|r, w| unsafe { w.bits(r.bits() & 0x7fffffff) });
    }
    /// Initialize the MAC.
    fn init_mac(wifi: &WIFI) {
        trace!("Initializing MAC.");
        wifi.ctrl()
            .modify(|r, w| unsafe { w.bits(r.bits() & 0xffffe800) });
    }
    /// Deinitialize the MAC.
    fn deinit_mac(wifi: &WIFI) {
        trace!("Deinitializing MAC.");
        wifi.ctrl().modify(|r, w| unsafe {
            w.bits(r.bits() | 0x17ff);
            while r.bits() & 0x2000 != 0 {}
            w
        });
    }
    /// Set the interrupt handler.
    fn set_isr() {
        trace!("Setting interrupt handler.");
        unsafe {
            map(
                Cpu::current(),
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
    fn ic_enable_rx(wifi: &WIFI) {
        trace!("Enabling RX.");
        wifi.rx_ctrl().write(|w| w.rx_enable().bit(true));
    }
    fn chip_enable(wifi: &WIFI) {
        trace!("chip_enable");
        Self::ic_enable_rx(wifi);
    }
    /// Initialize the WiFi peripheral.
    pub fn new<const BUFFER_SIZE: usize, const BUFFER_COUNT: usize>(
        wifi: WIFI,
        mut radio_clock: RADIO_CLK,
        _adc2: ADC2,
        dma_resources: &'res mut DMAResources<BUFFER_SIZE, BUFFER_COUNT>,
    ) -> Self {
        trace!("Initializing WiFi.");
        Self::enable_wifi_power_domain();
        Self::enable_clock(&mut radio_clock, RadioPeripherals::Wifi);
        Self::phy_enable(&mut radio_clock);
        let start_time = Instant::now();
        Self::reset_mac(&mut radio_clock, &wifi);
        Self::init_mac(&wifi);
        Self::ic_enable();
        Self::chip_enable(&wifi);
        let temp = Self {
            wifi,
            radio_clock,
            current_channel: AtomicU8::new(1),
            dma_list: Mutex::new(RefCell::new(DMAList::new(dma_resources))),
        };
        temp.set_channel(1).unwrap();
        WIFI_TX_SLOT_QUEUE.init(0..5);
        debug!(
            "WiFi MAC init complete. Took {} µs",
            start_time.elapsed().as_micros()
        );
        temp
    }
    /// Receive a frame.
    pub async fn receive<'a>(&'a self) -> BorrowedBuffer<'res, 'a> {
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
    /// Enable the transmission slot.
    fn enable_tx_slot(tx_slot_config: &TX_SLOT_CONFIG) {
        tx_slot_config
            .plcp0()
            .modify(|r, w| unsafe { w.bits(r.bits() | 0xc0000000) });
    }
    /// Invalidate the transmission slot.
    pub(crate) fn set_tx_slot_invalid(tx_slot_config: &TX_SLOT_CONFIG) {
        tx_slot_config
            .plcp0()
            .modify(|r, w| unsafe { w.bits(r.bits() | 0xb0000000) });
    }
    /// Set the packet for transmission.
    async fn transmit_internal(
        &self,
        dma_list_item: Pin<&TxDMAListItem>,
        rate: WiFiRate,
        slot: usize,
    ) -> WiFiResult<()> {
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
                .bits(rate.into_bits() & 0x1f)
                .unknown_enable()
                .bits(1)
        });
        tx_slot_parameters.plcp2().write(|w| w.unknown().bit(true));
        tx_slot_parameters
            .duration()
            .write(|w| unsafe { w.bits(0x0) });
        if rate.is_ht() {
            tx_slot_parameters.ht_sig().write(|w| unsafe {
                w.bits(
                    (rate.into_bits() as u32 & 0b111)
                        | ((length as u32 & 0xffff) << 8)
                        | (0b111 << 24)
                        | ((rate.is_short_gi() as u32) << 31),
                )
            });
            tx_slot_parameters
                .ht_unknown()
                .write(|w| unsafe { w.length().bits(length as u32 | 0x50000) });
        }
        /*
        tx_slot_config
            .config()
            .modify(|r, w| unsafe { w.bits(r.bits() | 0x02000000) });
        tx_slot_config
            .config()
            .modify(|r, w| unsafe { w.bits(r.bits() | 0x00003000) });
        */
        Self::enable_tx_slot(tx_slot_config);
        struct CancelOnDrop<'a> {
            tx_slot_config: &'a TX_SLOT_CONFIG,
            slot: usize,
        }
        impl CancelOnDrop<'_> {
            async fn wait_for_tx_complete(&self) -> WiFiResult<()> {
                // Wait for the hardware to confirm transmission.
                let res = match WIFI_TX_SLOTS[self.slot].wait().await {
                    TxSlotStatus::Done => Ok(()),
                    TxSlotStatus::Collision => Err(WiFiError::TxCollision),
                    TxSlotStatus::Timeout => Err(WiFiError::TxTimeout),
                };
                WIFI_TX_SLOTS[self.slot].reset();
                res
            }
            fn defuse(self) {
                mem::forget(self);
            }
        }
        impl Drop for CancelOnDrop<'_> {
            fn drop(&mut self) {
                WiFi::set_tx_slot_invalid(self.tx_slot_config);
                WIFI_TX_SLOTS[self.slot].reset();
            }
        }
        let cancel_on_drop = CancelOnDrop {
            tx_slot_config,
            slot,
        };
        let res = cancel_on_drop.wait_for_tx_complete().await;
        cancel_on_drop.defuse();

        res
    }
    /// Transmit a frame.
    ///
    /// The hardware expects the buffer to have room for an FCS. This means, the buffer passed will
    /// have to be four bytes longer than the actual payload. It doesn't matter if the FCS is
    /// actually already in the buffer, since the hardware will generate it.
    ///
    /// You must set a [TxErrorBehaviour], so the driver knows what to do in case of a TX error.
    /// The advantage of using this instead of bit banging a higher layer fix is, that we don't
    /// have to reacquire a TX slot every time TX fails.
    pub async fn transmit(
        &self,
        buffer: &[u8],
        rate: WiFiRate,
        error_behaviour: TxErrorBehaviour,
    ) -> WiFiResult<()> {
        let slot = WIFI_TX_SLOT_QUEUE.wait_for_slot().await;
        trace!("Acquired slot {}.", *slot);

        // We initialize the DMA list item.
        let mut dma_list_item = DMAListItem::new_for_tx(buffer);

        // And then pin it, before passing it to hardware.
        let dma_list_item = pin!(dma_list_item);
        let dma_list_ref = dma_list_item.into_ref();

        match error_behaviour {
            TxErrorBehaviour::Drop => self.transmit_internal(dma_list_ref, rate, *slot).await,
            TxErrorBehaviour::RetryUntil(retries) => {
                let mut res = self.transmit_internal(dma_list_ref, rate, *slot).await;
                if res.is_err() {
                    for _ in 0..retries {
                        res = self.transmit_internal(dma_list_ref, rate, *slot).await;
                        if res.is_ok() {
                            break;
                        }
                    }
                }

                res
            }
        }
    }
    /// Set the channel on which to operate.
    ///
    /// NOTE:
    /// This uses the proprietary blob.
    pub fn set_channel(&self, channel_number: u8) -> WiFiResult<()> {
        if !(1..=14).contains(&channel_number) {
            return Err(WiFiError::InvalidChannel);
        }
        trace!("Changing channel to {channel_number}");
        Self::deinit_mac(&self.wifi);
        unsafe {
            chip_v7_set_chan_nomac(channel_number, 0);
            disable_wifi_agc();
        }
        Self::init_mac(&self.wifi);
        unsafe {
            enable_wifi_agc();
        }
        self.current_channel
            .store(channel_number, Ordering::Relaxed);
        Ok(())
    }
    /// Returns the current channel.
    pub fn get_channel(&self) -> u8 {
        self.current_channel.load(Ordering::Relaxed)
    }
    pub fn set_filter_status(
        &self,
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
        &self,
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
        impl WiFi<'_> {
            $(
                pub fn $stat_name(&self) -> u32 {
                    self.wifi.$stat_name().read().bits()
                }
            )*
        }
    };
}
generate_stat_accessors![hw_stat_panic];
impl Drop for WiFi<'_> {
    fn drop(&mut self) {
        // Ensure, that the radio clocks and the power domain are disabled.
        Self::disable_clock(&mut self.radio_clock, RadioPeripherals::Wifi);
        Self::disable_clock(&mut self.radio_clock, RadioPeripherals::Phy);
        Self::disable_wifi_power_domain();
    }
}
