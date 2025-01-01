#![no_std]
pub(crate) mod fmt;

mod dma_list;
mod ffi;
mod phy_init_data;
mod wmac;

pub use dma_list::DMAResources;
pub use wmac::*;

#[cfg(not(feature = "critical_section"))]
type DefaultRawMutex = embassy_sync::blocking_mutex::raw::NoopRawMutex;
#[cfg(feature = "critical_section")]
type DefaultRawMutex = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
