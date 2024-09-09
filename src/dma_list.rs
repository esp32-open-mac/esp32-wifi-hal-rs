use core::{
    marker::{PhantomData, PhantomPinned},
    mem,
    ptr::{null_mut, NonNull},
    slice,
};
use log::{debug, trace};

use alloc::boxed::Box;
use bitfield_struct::bitfield;

use crate::regs::{
    base_rx_descr_ptr, last_rx_descr_ptr, next_rx_descr_ptr, MAC_BASE_RX_DESCR, MAC_RX_CTRL_REG,
};

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

pub struct Rx;
pub struct Tx;

#[repr(C)]
/// An entry into the [DMAList].
///
/// The type parameter allows differentiation between Rx and Tx [DMAListItem]s, to prevent the user from injection [DMAListItem]s, which aren't heap allocated.
pub struct DMAListItem<Use> {
    dma_list_header: DMAListHeader,
    buffer: NonNull<()>,
    next: *mut DMAListItem<Use>,
    _phantom: PhantomData<Use>,
    _phantom_pinned: PhantomPinned,
}
impl DMAListItem<Rx> {
    /// Initialize a new [DMAListItem] for RX.
    ///
    /// This is handled by the [DMAList].
    fn new_for_rx(buffer: &mut [u8], next: Option<NonNull<Self>>) -> Self {
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
            buffer: NonNull::new(buffer as *const _ as *mut u8 as _).unwrap(),
            next,
            _phantom: PhantomData,
            _phantom_pinned: PhantomPinned,
        }
    }
}
impl DMAListItem<Tx> {
    /// Initialize a new [DMAListItem] for TX.
    ///
    /// This is handled by [WiFi](crate::WiFi).
    pub fn new_for_tx(buffer: &[u8]) -> DMAListItem<Tx> {
        DMAListItem {
            dma_list_header: DMAListHeader::new()
                .with_buffer_size(buffer.len() as u16 + 32)
                .with_buffer_length(buffer.len() as u16)
                .with_has_data(true)
                .with_dma_owned(true),
            buffer: NonNull::new(buffer as *const _ as *mut ()).unwrap(),
            next: null_mut(),
            _phantom: PhantomData,
            _phantom_pinned: PhantomPinned,
        }
    }
}
impl<Use> DMAListItem<Use> {
    /// Returns a byte slice of the buffer, which is [DMAListHeader::buffer_length] long.
    pub fn buffer(&self) -> &[u8] {
        unsafe {
            slice::from_raw_parts(
                self.buffer.as_ptr() as _,
                self.dma_list_header.buffer_length() as usize,
            )
        }
    }
    fn next(&mut self) -> Option<NonNull<Self>> {
        NonNull::new(self.next)
    }
    fn set_next(&mut self, next: Option<&mut Self>) {
        self.next = match next {
            Some(next) => next,
            None => null_mut(),
        };
    }
}
pub type RxDMAListItem = DMAListItem<Rx>;
pub type TxDMAListItem = DMAListItem<Tx>;
pub struct DMAList {
    rx_chain_ptrs: Option<(NonNull<RxDMAListItem>, NonNull<RxDMAListItem>)>,
}
impl DMAList {
    pub fn set_rx_base_addr(item: Option<NonNull<RxDMAListItem>>) {
        unsafe {
            MAC_BASE_RX_DESCR.write_volatile(match item {
                Some(dma_list_descriptor) => dma_list_descriptor.as_ptr(),
                None => null_mut(),
            })
        }
    }
    /// Allocates all buffers for the DMA list.
    pub fn allocate(buffer_count: usize) -> Self {
        assert!(buffer_count > 0);
        // Due to the assert, this will definitely be not null.
        let mut rx_chain_last = None;
        let mut prev = None;

        for _ in 0..buffer_count {
            let buffer = Box::leak(Box::new([0u8; 1600]));
            let item =
                NonNull::new(Box::leak(Box::new(DMAListItem::new_for_rx(buffer, prev)))).unwrap();
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
    /// Tells the hardware about our DMA list.
    ///
    /// This was separated out from [Self::allocate], this is owned by the [crate::WiFi] struct and has to be initialized at the end.
    pub fn init(&mut self) {
        debug!("Initialized DMA list.");
        Self::set_rx_base_addr(self.rx_chain_ptrs.map(|rx_chain_ptrs| rx_chain_ptrs.0));
        Self::log_stats();
    }
    /// Tell the hardware to reload the DMA list.
    ///
    /// This will update [MAC_NEXT_RX_DESCR] and [MAC_LAST_RX_DESCR].
    fn reload_rx_descriptors() {
        trace!("Reloading RX descriptors.");
        unsafe {
            MAC_RX_CTRL_REG.write_volatile(MAC_RX_CTRL_REG.read_volatile() | 1);
            while MAC_RX_CTRL_REG.read_volatile() & 1 != 0 {}
        }
    }
    /// Sets [Self::rx_chain_ptrs], with the `dma_list_descriptor` at the base.
    fn set_rx_chain_begin(&mut self, dma_list_descriptor: Option<NonNull<RxDMAListItem>>) {
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
    /// Take the first [DMAListItem] out of the list.
    pub fn take_first<'a>(&mut self) -> Option<&'a mut RxDMAListItem> {
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
    /// Returns a [DMAListItem] to the end of the list.
    pub fn recycle(&mut self, dma_list_descriptor: &mut RxDMAListItem) {
        dma_list_descriptor.dma_list_header.set_has_data(false);
        dma_list_descriptor
            .dma_list_header
            .set_buffer_length(dma_list_descriptor.dma_list_header.buffer_size());
        if let Some(ref mut rx_chain_ptrs) = self.rx_chain_ptrs {
            unsafe { rx_chain_ptrs.1.as_mut() }.set_next(Some(dma_list_descriptor));
            Self::reload_rx_descriptors();

            let last_descr = last_rx_descr_ptr();
            if base_rx_descr_ptr() as u32 == 0x3ff00000 {
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
        let (rx_next, rx_last) = (next_rx_descr_ptr() as u32, last_rx_descr_ptr() as u32);
        trace!("DMA list: Next: {rx_next:x} Last: {rx_last:x}");
    }
}
impl Drop for DMAList {
    fn drop(&mut self) {
        while let Some(dma_list_descriptor) = self.take_first() {
            // SAFETY:
            // We know, that only DMAList can allocate these, and it always does so through a Box.
            // Therefore these pointer definitely point to memory, that was allocated through said Box.
            unsafe {
                mem::drop(Box::from_raw(dma_list_descriptor.next));
                mem::drop(Box::from_raw(dma_list_descriptor));
            }
        }
    }
}
