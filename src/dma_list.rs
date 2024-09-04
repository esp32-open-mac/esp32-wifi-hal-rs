use core::{
    ptr::{null_mut, NonNull},
    slice,
};
use log::{debug, trace};

use alloc::boxed::Box;
use bitfield_struct::bitfield;

use crate::regs::{MAC_BASE_RX_DESCR, MAC_LAST_RX_DESCR, MAC_NEXT_RX_DESCR, MAC_RX_CTRL_REG};

#[bitfield(u32)]
pub struct DMAListHeader {
    #[bits(12)]
    pub buffer_size: u16,
    #[bits(12)]
    pub buffer_length: u16,
    #[bits(6)]
    pub __: u8,
    pub has_data: bool,
    pub dma_owned: bool,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct DMAListItem {
    dma_list_header: DMAListHeader,
    buffer: NonNull<()>,
    next: *mut DMAListItem,
}
impl DMAListItem {
    /// Returns a byte slice of the buffer, which is [DMAListHeader::buffer_length] long.
    pub fn buffer(&self) -> &[u8] {
        unsafe {
            slice::from_raw_parts(
                self.buffer.as_ptr() as _,
                self.dma_list_header.buffer_length() as usize,
            )
        }
    }
    pub fn next(&mut self) -> Option<NonNull<DMAListItem>> {
        NonNull::new(self.next)
    }
    pub fn new(buffer: &mut [u8], next: Option<NonNull<DMAListItem>>) -> Self {
        let next = match next {
            Some(next) => next.as_ptr(),
            None => null_mut(),
        };
        Self {
            dma_list_header: DMAListHeader::new()
                .with_buffer_size(buffer.len() as u16)
                .with_buffer_length(buffer.len() as u16)
                .with_has_data(false)
                .with_dma_owned(true),
            buffer: NonNull::new(buffer as *mut _ as _).unwrap(),
            next,
        }
    }
    pub fn set_next(&mut self, next: Option<&mut DMAListItem>) {
        self.next = match next {
            Some(next) => next,
            None => null_mut(),
        };
    }
}
pub struct DMAList {
    rx_chain_ptrs: Option<(NonNull<DMAListItem>, NonNull<DMAListItem>)>,
}
impl DMAList {
    pub fn set_rx_base_addr(item: Option<NonNull<DMAListItem>>) {
        unsafe {
            MAC_BASE_RX_DESCR.write_volatile(match item {
                Some(dma_list_descriptor) => dma_list_descriptor.as_ptr(),
                None => null_mut(),
            })
        }
    }
    pub fn allocate(buffer_count: usize) -> Self {
        assert!(buffer_count > 0);
        // Due to the assert, this will definitely be not null.
        let mut rx_chain_last = None;
        let mut prev = None;

        for _ in 0..buffer_count {
            let buffer = Box::leak(Box::new([0u8; 1600]));
            let item = NonNull::new(Box::leak(Box::new(DMAListItem::new(buffer, prev)))).unwrap();
            if prev.is_none() {
                rx_chain_last = Some(item);
            }
            prev = Some(item);
        }
        let rx_chain_begin = prev.unwrap();
        let rx_chain_last = rx_chain_last.unwrap();
        debug!("Allocated DMA list with {buffer_count} buffers.");
        Self {
            rx_chain_ptrs: Some((rx_chain_begin, rx_chain_last)),
        }
    }
    pub fn init(&mut self) {
        debug!("Initialized DMA list.");
        Self::set_rx_base_addr(self.rx_chain_ptrs.map(|rx_chain_ptrs| rx_chain_ptrs.0));
        Self::log_stats();
    }
    pub fn reload_rx_descriptors() {
        trace!("Reloading RX descriptors.");
        unsafe {
            MAC_RX_CTRL_REG.write_volatile(MAC_RX_CTRL_REG.read_volatile() | 1);
            while MAC_RX_CTRL_REG.read_volatile() & 1 != 0 {}
        }
    }
    fn set_rx_chain_begin(&mut self, dma_list_descriptor: Option<NonNull<DMAListItem>>) {
        match (&mut self.rx_chain_ptrs, dma_list_descriptor) {
            // If neither the DMA list nor the DMA list descriptor is empty, we simply set rx_chain_begin to dma_list_desciptor.
            (Some(rx_chain_ptrs), Some(dma_list_descriptor)) => {
                rx_chain_ptrs.0 = dma_list_descriptor;
            }
            // If the DMA list isn't empty, but we want to set it to empty.
            (Some(_), None) => self.rx_chain_ptrs = None,
            // The DMA list is currently empty. Therefore the dma_list_descriptor is now first and last.
            (None, Some(dma_list_descriptor)) => {
                self.rx_chain_ptrs = Some((dma_list_descriptor, dma_list_descriptor))
            }
            _ => {}
        }
        Self::set_rx_base_addr(dma_list_descriptor)
    }
    pub fn take_first<'a>(&mut self) -> Option<&'a mut DMAListItem> {
        let first = unsafe { self.rx_chain_ptrs.unwrap().0.as_mut() };
        trace!("Taking buffer: {:x} from DMA list.", first as *mut _ as u32);
        if first.dma_list_header.has_data() {
            let next = first.next();
            if next.is_none() {
                trace!("Next was none.");
            }
            self.set_rx_chain_begin(next);
            Some(first)
        } else {
            None
        }
    }
    pub fn recycle(&mut self, dma_list_descriptor: &mut DMAListItem) {
        dma_list_descriptor.dma_list_header.set_has_data(false);
        dma_list_descriptor
            .dma_list_header
            .set_buffer_length(dma_list_descriptor.dma_list_header.buffer_size());
        if let Some(ref mut rx_chain_ptrs) = self.rx_chain_ptrs {
            unsafe { rx_chain_ptrs.1.as_mut() }.set_next(Some(dma_list_descriptor));
            Self::reload_rx_descriptors();

            let last_descr = unsafe { MAC_LAST_RX_DESCR.read_volatile() };
            if unsafe { MAC_NEXT_RX_DESCR.read_volatile() } as u32 == 0x3ff00000 {
                trace!("Got weird value.");
                if last_descr == dma_list_descriptor {
                    Self::set_rx_base_addr(unsafe { last_descr.as_mut().unwrap() }.next());
                }
            }
            rx_chain_ptrs.1 = NonNull::new(dma_list_descriptor).unwrap();
        } else {
            self.set_rx_chain_begin(NonNull::new(dma_list_descriptor));
        }
        trace!(
            "Returned buffer: {:x} to DMA list.",
            dma_list_descriptor as *mut _ as u32
        );
    }
    pub fn log_stats() {
        let (rx_next, rx_last) = unsafe {
            (
                MAC_NEXT_RX_DESCR.read_volatile() as u32,
                MAC_LAST_RX_DESCR.read_volatile() as u32,
            )
        };
        trace!("DMA list: Next: {rx_next:x} Last: {rx_last:x}");
    }
}
