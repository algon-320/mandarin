#![allow(dead_code)]

use core::mem::MaybeUninit;
use core::mem::{align_of, size_of};
use core::ptr::NonNull;

#[repr(C, align(4096))]
pub struct StaticMallocator<const BUF_SIZE: usize> {
    buf: MaybeUninit<[u8; BUF_SIZE]>,
    ptr: usize,
    end: usize,
    initialized: bool,
}
impl<const BUF_SIZE: usize> StaticMallocator<BUF_SIZE> {
    pub const fn new() -> Self {
        Self {
            buf: MaybeUninit::uninit(),
            ptr: 0,
            end: 0,
            initialized: false,
        }
    }
    pub fn ensure_initialized(&mut self) {
        if self.initialized {
            return;
        }

        let ptr = self.buf.as_mut_ptr();
        unsafe { (ptr as *mut u8).write_bytes(0, BUF_SIZE) };
        self.ptr = ptr as usize;
        self.end = self.ptr + BUF_SIZE;
        self.initialized = true;
    }

    fn ceil(addr: usize, align: usize) -> usize {
        (addr + align - 1) & !(align - 1)
    }

    pub fn alloc(&mut self, size: usize, align: usize, boundary: usize) -> Option<NonNull<u8>> {
        self.ensure_initialized();

        let mut ptr = Self::ceil(self.ptr, align);
        let next_boundary = Self::ceil(self.ptr, boundary);
        if next_boundary < ptr + size {
            ptr = next_boundary;
        }
        if self.end < ptr + size {
            None
        } else {
            trace!("memory allocated: start={:#x}, size={:#x}", ptr, size);
            self.ptr = ptr + size;
            Some(unsafe { NonNull::new_unchecked(ptr as *mut u8) })
        }
    }

    const PAGE_BOUNDARY: usize = 4096;

    pub fn alloc_slice<T>(&mut self, len: usize) -> Option<NonNull<[MaybeUninit<T>]>> {
        let ptr = self.alloc(size_of::<T>() * len, align_of::<T>(), Self::PAGE_BOUNDARY)?;
        Some(unsafe {
            NonNull::new_unchecked(core::ptr::slice_from_raw_parts_mut(
                ptr.as_ptr() as *mut MaybeUninit<T>,
                len,
            ))
        })
    }
    pub fn alloc_slice_ext<T>(
        &mut self,
        len: usize,
        align: usize,
        boundary: usize,
    ) -> Option<NonNull<[MaybeUninit<T>]>> {
        let ptr = self.alloc(size_of::<T>() * len, align, boundary)?;
        Some(unsafe {
            NonNull::new_unchecked(core::ptr::slice_from_raw_parts_mut(
                ptr.as_ptr() as *mut MaybeUninit<T>,
                len,
            ))
        })
    }

    pub fn alloc_obj<T>(&mut self) -> Option<NonNull<MaybeUninit<T>>> {
        let ptr = self.alloc(size_of::<T>(), align_of::<T>(), Self::PAGE_BOUNDARY)?;
        Some(unsafe { NonNull::new_unchecked(ptr.as_ptr() as *mut MaybeUninit<T>) })
    }
    pub fn alloc_obj_ext<T>(
        &mut self,
        align: usize,
        boundary: usize,
    ) -> Option<NonNull<MaybeUninit<T>>> {
        let ptr = self.alloc(size_of::<T>(), align, boundary)?;
        Some(unsafe { NonNull::new_unchecked(ptr.as_ptr() as *mut MaybeUninit<T>) })
    }
}

pub struct FixedVec<T, const CAPACITY: usize> {
    buf: MaybeUninit<[T; CAPACITY]>,
    len: usize,
}
impl<T, const CAPACITY: usize> FixedVec<T, CAPACITY> {
    pub const fn new() -> Self {
        Self {
            buf: MaybeUninit::uninit(),
            len: 0,
        }
    }
    pub fn capacity(&self) -> usize {
        CAPACITY
    }
    pub fn len(&self) -> usize {
        self.len
    }
    pub fn as_slice(&self) -> &[T] {
        let p = self.buf.as_ptr() as *const T;
        unsafe { core::slice::from_raw_parts(p, self.len) }
    }
    pub fn get(&self, idx: usize) -> Option<&T> {
        if idx < self.len {
            let p = self.buf.as_ptr();
            Some(unsafe { &*(p as *const T).add(idx) })
        } else {
            None
        }
    }
    pub fn get_mut(&mut self, idx: usize) -> Option<&mut T> {
        if idx < self.len {
            let p = self.buf.as_mut_ptr();
            Some(unsafe { &mut *(p as *mut T).add(idx) })
        } else {
            None
        }
    }
    pub fn push(&mut self, val: T) -> Option<usize> {
        if self.len < CAPACITY {
            let p = self.buf.as_mut_ptr();
            let idx = self.len;
            unsafe { (p as *mut T).add(idx).write(val) };
            self.len += 1;
            Some(idx)
        } else {
            None
        }
    }
    pub fn pop(&mut self) -> Option<T> {
        if self.len > 0 {
            self.len -= 1;
            let p = self.buf.as_mut_ptr();
            Some(unsafe { (p as *const T).add(self.len).read() })
        } else {
            None
        }
    }
    pub fn clear(&mut self) {
        while let Some(x) = self.pop() {
            drop(x);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test_case]
    fn test_fixed_vec() {
        static mut FIXED_VEC: FixedVec<i32, 10> = FixedVec::new();
        unsafe {
            assert_eq!(FIXED_VEC.len(), 0);

            assert_eq!(FIXED_VEC.push(1), Some(0));
            assert_eq!(FIXED_VEC.len(), 1);

            assert_eq!(FIXED_VEC.push(2), Some(1));
            assert_eq!(FIXED_VEC.len(), 2);

            let x = FIXED_VEC.pop();
            assert_eq!(FIXED_VEC.len(), 1);
            assert_eq!(x, Some(2));

            let x = FIXED_VEC.pop();
            assert_eq!(FIXED_VEC.len(), 0);
            assert_eq!(x, Some(1));

            let x = FIXED_VEC.pop();
            assert_eq!(FIXED_VEC.len(), 0);
            assert_eq!(x, None);

            FIXED_VEC.push(0);
            FIXED_VEC.push(1);
            FIXED_VEC.push(2);
            FIXED_VEC.push(3);
            FIXED_VEC.push(4);
            assert_eq!(FIXED_VEC.len(), 5);
            assert_eq!(FIXED_VEC.get(0), Some(&0));
            assert_eq!(FIXED_VEC.get(4), Some(&4));
            assert_eq!(FIXED_VEC.get(5), None);

            *FIXED_VEC.get_mut(3).unwrap() += 10;
            assert_eq!(FIXED_VEC.get(3), Some(&13));

            FIXED_VEC.clear();
            assert_eq!(FIXED_VEC.len(), 0);
            for i in 0..10 {
                assert_eq!(FIXED_VEC.push(i), Some(i as usize));
            }
            assert_eq!(FIXED_VEC.len(), 10);
            assert_eq!(FIXED_VEC.push(10), None);
            assert_eq!(FIXED_VEC.push(11), None);
        }
    }
}

#[cfg(test)]
mod malloc_tests {
    use super::*;
    #[test_case]
    fn test_alloc() {
        const BUF_SIZE: usize = 1024 * 4;
        static mut MALLOC: StaticMallocator<BUF_SIZE> = StaticMallocator::new();
        unsafe {
            let buf_ptr = MALLOC.buf.as_mut_ptr() as usize;

            let ptr = MALLOC.alloc_obj::<u8>();
            assert_eq!(ptr.unwrap().as_ptr() as usize, buf_ptr);

            let ptr = MALLOC.alloc_obj::<u8>();
            assert_eq!(ptr.unwrap().as_ptr() as usize, buf_ptr + 1);

            let ptr = MALLOC.alloc(1, 8, 1024);
            assert_eq!(ptr.unwrap().as_ptr() as usize, buf_ptr + 8);

            let ptr = MALLOC.alloc(1, 8, 1024);
            assert_eq!(ptr.unwrap().as_ptr() as usize, buf_ptr + 16);

            let ptr = MALLOC.alloc(1, 32, 1024);
            assert_eq!(ptr.unwrap().as_ptr() as usize, buf_ptr + 32);

            let ptr = MALLOC.alloc(1, 32, 1024);
            assert_eq!(ptr.unwrap().as_ptr() as usize, buf_ptr + 64);

            let ptr = MALLOC.alloc(62, 1, 1024);
            assert_eq!(ptr.unwrap().as_ptr() as usize, buf_ptr + 65);

            let ptr = MALLOC.alloc(4, 1, 128);
            assert_eq!(ptr.unwrap().as_ptr() as usize, buf_ptr + 128);

            let ptr = MALLOC.alloc(64, 1024, 1024);
            assert_eq!(ptr.unwrap().as_ptr() as usize, buf_ptr + 1024);
        }
    }
    #[test_case]
    fn test_alloc_obj() {
        const BUF_SIZE: usize = 1024 * 4;
        static mut MALLOC: StaticMallocator<BUF_SIZE> = StaticMallocator::new();
        unsafe {
            let buf_ptr = MALLOC.buf.as_mut_ptr() as usize;

            let ptr = MALLOC.alloc(1, 1, 1024);
            assert_eq!(ptr.unwrap().as_ptr() as usize, buf_ptr + 0);

            #[repr(align(64))]
            struct Foo(u32);

            let ptr = MALLOC.alloc_obj::<Foo>();
            assert_eq!(ptr.unwrap().as_ptr() as usize, buf_ptr + 64);
        }
    }
}
