use super::StaticMallocator;
use core::ptr::NonNull;
use core::slice::{from_raw_parts, from_raw_parts_mut, SliceIndex};

pub struct Buffer {
    ptr: Option<NonNull<u8>>,
    size: usize,
}
impl Buffer {
    pub fn new<const SIZE: usize>(
        malloc: &mut StaticMallocator<SIZE>,
        size: usize,
        align: usize,
    ) -> Self {
        let buf = unsafe {
            malloc
                .alloc(size, align, None)
                .expect("memory shortage")
                .as_mut()
        };
        Self {
            ptr: Some(unsafe { NonNull::new_unchecked(buf.as_mut_ptr()) }),
            size,
        }
    }

    pub fn detach(&mut self) -> NonNull<u8> {
        self.ptr.take().expect("ownership error")
    }

    /// Safety: `ptr` must be a pointer derived from `self.detach`.
    pub unsafe fn attach(&mut self, ptr: NonNull<u8>) {
        debug_assert!(self.ptr.is_none());
        self.ptr = Some(ptr);
    }

    pub fn own(&self) -> bool {
        self.ptr.is_some()
    }
}

impl<I> core::ops::Index<I> for Buffer
where
    I: SliceIndex<[u8], Output = [u8]>,
{
    type Output = [u8];
    fn index(&self, range: I) -> &Self::Output {
        let ptr = self.ptr.expect("ownership error");
        unsafe { &from_raw_parts(ptr.as_ptr(), self.size)[range] }
    }
}
impl<I> core::ops::IndexMut<I> for Buffer
where
    I: SliceIndex<[u8], Output = [u8]>,
{
    fn index_mut(&mut self, range: I) -> &mut Self::Output {
        let ptr = self.ptr.expect("ownership error");
        unsafe { &mut from_raw_parts_mut(ptr.as_ptr(), self.size)[range] }
    }
}
