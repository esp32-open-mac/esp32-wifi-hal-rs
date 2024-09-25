use crate::dma_list::RxDMAListItem;

pub const MAC_CTRL_REG: *mut u32 = 0x3ff73cb8 as _;
pub const MAC_RX_CTRL_REG: *mut u32 = 0x3ff73084 as _;

pub const MAC_DMA_INT_STATUS: *mut u32 = 0x3ff73c48 as _;
pub const MAC_DMA_INT_CLEAR: *mut u32 = 0x3ff73c4c as _;

pub const MAC_BASE_RX_DESCR: *mut *mut RxDMAListItem = 0x3ff73088 as _;
pub const MAC_NEXT_RX_DESCR: *mut *mut RxDMAListItem = 0x3ff7308c as _;
pub const MAC_LAST_RX_DESCR: *mut *mut RxDMAListItem = 0x3ff73090 as _;
pub fn base_rx_descr_ptr() -> *mut RxDMAListItem {
    unsafe { MAC_BASE_RX_DESCR.read_volatile() }
}
pub fn next_rx_descr_ptr() -> *mut RxDMAListItem {
    unsafe { MAC_NEXT_RX_DESCR.read_volatile() }
}
pub fn last_rx_descr_ptr() -> *mut RxDMAListItem {
    unsafe { MAC_LAST_RX_DESCR.read_volatile() }
}

const MAC_TX_CONFIG_BASE: *mut u32 = 0x3ff73d1c as _;
const MAC_TX_CONFIG_OFFSET: usize = 2;
const MAC_TX_PLCP0_BASE: *mut u32 = 0x3ff73d20 as _;
const MAC_TX_PLCP0_OFFSET: usize = 2;
const MAC_TX_PLCP1_BASE: *mut u32 = 0x3ff74258 as _;
const MAC_TX_PLCP1_OFFSET: usize = 0xf;
const MAC_TX_PLCP2_BASE: *mut u32 = 0x3ff7425c as _;
const MAC_TX_PLCP2_OFFSET: usize = 0xf;
const MAC_TX_HT_SIG_BASE: *mut u32 = 0x3ff74260 as _;
const MAC_TX_HT_SIG_OFFSET: usize = 0xf;
const MAC_TX_HT_UNKNOWN_BASE: *mut u32 = 0x3ff74264 as _;
const MAC_TX_HT_UNKNOWN_OFFSET: usize = 0xf;
const MAC_TX_DURATION_BASE: *mut u32 = 0x3ff74268 as _;
const MAC_TX_DURATION_OFFSET: usize = 0xf;

pub const MAC_TXQ_COMPLETE_STATUS: *mut u32 = 0x3ff73cc8 as _;
pub const MAC_TXQ_COMPLETE_CLEAR: *mut u32 = 0x3ff73cc4 as _;
// The two LSBs are for collisions and the two MSBs are for timeouts.
pub const MAC_TXQ_ERROR_STATUS: *mut u32 = 0x3ff73cc0 as _;
pub const MAC_TXQ_ERROR_CLEAR: *mut u32 = 0x3ff73cbc as _;

pub const fn tx_config(slot: usize) -> *mut u32 {
    unsafe { MAC_TX_CONFIG_BASE.sub(MAC_TX_CONFIG_OFFSET * slot) }
}
pub const fn plcp0(slot: usize) -> *mut u32 {
    unsafe { MAC_TX_PLCP0_BASE.sub(MAC_TX_PLCP0_OFFSET * slot) }
}
pub const fn plcp1(slot: usize) -> *mut u32 {
    unsafe { MAC_TX_PLCP1_BASE.sub(MAC_TX_PLCP1_OFFSET * slot) }
}
pub const fn plcp2(slot: usize) -> *mut u32 {
    unsafe { MAC_TX_PLCP2_BASE.sub(MAC_TX_PLCP2_OFFSET * slot) }
}
pub const fn ht_sig(slot: usize) -> *mut u32 {
    unsafe { MAC_TX_HT_SIG_BASE.sub(MAC_TX_HT_SIG_OFFSET * slot) }
}
pub const fn ht_unknown(slot: usize) -> *mut u32 {
    unsafe { MAC_TX_HT_UNKNOWN_BASE.sub(MAC_TX_HT_UNKNOWN_OFFSET * slot) }
}
pub const fn duration(slot: usize) -> *mut u32 {
    unsafe { MAC_TX_DURATION_BASE.sub(MAC_TX_DURATION_OFFSET * slot) }
}
/*
const MAC_INTERRUPT_CTRL: *mut u32 = 0x3ff73c4c as _;
const UNKNOWN_MAC_INIT_ZERO: *mut u32 = 0x3ff73d24 as _;
const UNKNOWN_MAC_INIT_ONE: *mut u32 = 0x3ff73d24 as _;
const UNKNOWN_MAC_SLEEP_ZERO: *mut u32 = 0x3ff73c40 as _; */
