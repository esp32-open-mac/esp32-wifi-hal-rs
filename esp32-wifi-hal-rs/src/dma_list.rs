use core::{
    marker::{PhantomData, PhantomPinned},
    ptr::{addr_of_mut, null_mut, NonNull},
    slice,
};
use esp32::WIFI;

use bitfield_struct::bitfield;

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
/// The type parameter allows differentiation between Rx and Tx [DMAListItem]s, to prevent the user from injecting [DMAListItem]s, which aren't statically allocated.
pub struct DMAListItem<Use> {
    dma_list_header: DMAListHeader,
    buffer: *mut u8,
    next: *mut DMAListItem<Use>,
    _phantom: PhantomData<Use>,
    _phantom_pinned: PhantomPinned,
}
unsafe impl<Use> Sync for DMAListItem<Use> {}
unsafe impl<Use> Send for DMAListItem<Use> {}
impl DMAListItem<Rx> {
    /// Initialize a new [DMAListItem] for RX.
    ///
    /// This is handled by the [DMAList].
    fn init_for_rx(&mut self, buffer: &mut [u8], next: Option<NonNull<Self>>) {
        let next = match next {
            Some(next) => next.as_ptr(),
            None => null_mut(),
        };
        self.buffer = buffer as *const _ as *mut u8;
        self.next = next;
        self.dma_list_header = DMAListHeader::new()
            .with_buffer_size(buffer.len() as u16)
            .with_buffer_length(buffer.len() as u16)
            .with_has_data(false)
            .with_dma_owned(true);
    }
}
impl DMAListItem<Tx> {
    /// Initialize a new [DMAListItem] for TX.
    ///
    /// This is handled by [WiFi](crate::WiFi).
    pub fn new_for_tx(buffer: &[u8]) -> DMAListItem<Tx> {
        let mut temp = Self::UNINIT;
        temp.buffer = buffer as *const _ as *mut u8 as _;
        temp.next = null_mut();
        temp.dma_list_header = DMAListHeader::new()
            .with_buffer_size(buffer.len() as u16)
            .with_buffer_length(buffer.len() as u16)
            .with_has_data(true)
            .with_dma_owned(true);
        temp
    }
}
impl<Use> DMAListItem<Use> {
    pub const UNINIT: Self = Self {
        dma_list_header: DMAListHeader::new(),
        buffer: null_mut(),
        next: null_mut(),
        _phantom: PhantomData,
        _phantom_pinned: PhantomPinned,
    };
    /// Returns a byte slice of the buffer, which is [DMAListHeader::buffer_length] long.
    pub fn buffer(&self) -> &[u8] {
        assert!(!self.buffer.is_null());
        unsafe {
            slice::from_raw_parts(
                self.buffer as _,
                self.dma_list_header.buffer_length() as usize,
            )
        }
    }
    fn next(&mut self) -> Option<NonNull<Self>> {
        NonNull::new(self.next)
    }
    fn set_next(&mut self, next: Option<*mut Self>) {
        self.next = match next {
            Some(next) => next,
            None => null_mut(),
        };
    }
}
pub type RxDMAListItem = DMAListItem<Rx>;
pub type TxDMAListItem = DMAListItem<Tx>;
pub struct DMAResources<const BUFFER_SIZE: usize, const BUFFER_COUNT: usize> {
    buffers: [[u8; BUFFER_SIZE]; BUFFER_COUNT],
    dma_list_items: [RxDMAListItem; BUFFER_COUNT],
}
impl<const BUFFER_SIZE: usize, const BUFFER_COUNT: usize> DMAResources<BUFFER_SIZE, BUFFER_COUNT> {
    pub const fn new() -> Self {
        Self {
            buffers: [[0u8; BUFFER_SIZE]; BUFFER_COUNT],
            dma_list_items: [RxDMAListItem::UNINIT; BUFFER_COUNT],
        }
    }
    fn init(&'static mut self) -> (NonNull<RxDMAListItem>, NonNull<RxDMAListItem>) {
        for (i, dma_list_item) in self.dma_list_items.iter_mut().enumerate() {
            dma_list_item.init_for_rx(&mut self.buffers[i], None);
        }
        for i in 0..(BUFFER_COUNT - 1) {
            let next = addr_of_mut!(self.dma_list_items[i + 1]);
            self.dma_list_items[i].set_next(Some(next));
        }
        (
            NonNull::new(addr_of_mut!(self.dma_list_items[0])).unwrap(),
            NonNull::new(addr_of_mut!(self.dma_list_items[BUFFER_COUNT - 1])).unwrap(),
        )
    }
}
impl<const BUFFER_SIZE: usize, const BUFFER_COUNT: usize> Default
    for DMAResources<BUFFER_SIZE, BUFFER_COUNT>
{
    fn default() -> Self {
        Self::new()
    }
}

pub struct DMAList {
    rx_chain_ptrs: Option<(NonNull<RxDMAListItem>, NonNull<RxDMAListItem>)>,
}
impl DMAList {
    /// Tell the hardware to reload the DMA list.
    ///
    /// This will update [MAC_NEXT_RX_DESCR] and [MAC_LAST_RX_DESCR].
    fn reload_rx_descriptors() {
        trace!("Reloading RX descriptors.");
        unsafe { WIFI::steal() }.rx_ctrl().modify(|r, w| {
            w.rx_descr_reload().bit(true);
            while r.rx_descr_reload().bit() {}
            w
        });
    }
    pub fn set_rx_base_addr(item: Option<NonNull<RxDMAListItem>>) {
        unsafe { WIFI::steal() }.rx_descr_base().write(|w| unsafe {
            w.bits(match item {
                Some(dma_list_descriptor) => dma_list_descriptor.as_ptr(),
                None => null_mut(),
            } as _)
        });
    }
    fn last_rx_descr_ptr() -> *mut RxDMAListItem {
        unsafe { WIFI::steal() }.rx_descr_last().read().bits() as _
    }
    fn next_rx_descr_ptr() -> *mut RxDMAListItem {
        unsafe { WIFI::steal() }.rx_descr_next().read().bits() as _
    }
    fn base_rx_descr_ptr() -> *mut RxDMAListItem {
        unsafe { WIFI::steal() }.rx_descr_base().read().bits() as _
    }
    /// Instantiate a new DMAList.
    pub fn new<const BUFFER_SIZE: usize, const BUFFER_COUNT: usize>(
        resources: &'static mut DMAResources<BUFFER_SIZE, BUFFER_COUNT>,
    ) -> Self {
        let rx_chain_ptrs = resources.init();
        Self::set_rx_base_addr(Some(rx_chain_ptrs.0));
        debug!("Initialized DMA list.");
        Self::log_stats();
        Self {
            rx_chain_ptrs: Some(rx_chain_ptrs),
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

            let last_descr = Self::last_rx_descr_ptr();
            if Self::base_rx_descr_ptr() as u32 == 0x3ff00000 {
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
        #[allow(unused)]
        let (rx_next, rx_last) = (
            Self::next_rx_descr_ptr() as u32,
            Self::last_rx_descr_ptr() as u32,
        );
        debug!("DMA list: Next: {rx_next:x} Last: {rx_last:x}");
    }
}
unsafe impl Sync for DMAList {}
unsafe impl Send for DMAList {}
