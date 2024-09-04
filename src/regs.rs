use crate::dma_list::DMAListItem;

pub const MAC_CTRL_REG: *mut u32 = 0x3ff73cb8 as _;
pub const MAC_RX_CTRL_REG: *mut u32 = 0x3ff73084 as _;
pub const MAC_DMA_INT_STATUS: *mut u32 = 0x3ff73c48 as _;
pub const MAC_DMA_INT_CLEAR: *mut u32 = 0x3ff73c4c as _;
pub const MAC_BASE_RX_DESCR: *mut *mut DMAListItem = 0x3ff73088 as _;
pub const MAC_NEXT_RX_DESCR: *mut *mut DMAListItem = 0x3ff7308c as _;
pub const MAC_LAST_RX_DESCR: *mut *mut DMAListItem = 0x3ff73090 as _;
/*
const MAC_INTERRUPT_CTRL: *mut u32 = 0x3ff73c4c as _;
const UNKNOWN_MAC_INIT_ZERO: *mut u32 = 0x3ff73d24 as _;
const UNKNOWN_MAC_INIT_ONE: *mut u32 = 0x3ff73d24 as _;
const UNKNOWN_MAC_SLEEP_ZERO: *mut u32 = 0x3ff73c40 as _; */
