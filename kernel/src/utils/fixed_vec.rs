#![allow(dead_code)]

use core::mem::MaybeUninit;

pub struct FixedVec<T, const CAPACITY: usize> {
    buf: [MaybeUninit<T>; CAPACITY],
    len: usize,
}
pub struct FixedVecIter<'buf, T> {
    buf: &'buf [MaybeUninit<T>],
    idx: usize,
}
pub struct FixedVecIterMut<'buf, T> {
    buf: &'buf mut [MaybeUninit<T>],
    idx: usize,
}

impl<T, const CAPACITY: usize> FixedVec<T, CAPACITY> {
    pub const fn new() -> Self {
        Self {
            buf: unsafe { MaybeUninit::uninit().assume_init() },
            len: 0,
        }
    }
    pub unsafe fn initialize_ptr(ptr: *mut Self) {
        let len = core::ptr::addr_of_mut!((*ptr).len);
        len.write(0);
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
    pub fn iter(&self) -> FixedVecIter<'_, T> {
        FixedVecIter {
            buf: &self.buf[..self.len],
            idx: 0,
        }
    }
    pub fn iter_mut(&mut self) -> FixedVecIterMut<'_, T> {
        FixedVecIterMut {
            buf: &mut self.buf[..self.len],
            idx: 0,
        }
    }
}

impl<'buf, T> Iterator for FixedVecIter<'buf, T> {
    type Item = &'buf T;
    fn next(&mut self) -> Option<Self::Item> {
        if self.idx == self.buf.len() {
            None
        } else {
            let ret = Some(unsafe { &*self.buf[self.idx].as_ptr() });
            self.idx += 1;
            ret
        }
    }
}
impl<'buf, T> Iterator for FixedVecIterMut<'buf, T> {
    type Item = &'buf mut T;
    fn next(&mut self) -> Option<Self::Item> {
        if self.idx == self.buf.len() {
            None
        } else {
            let ret = Some(unsafe { &mut *self.buf[self.idx].as_mut_ptr() });
            self.idx += 1;
            ret
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
    #[test_case]
    fn test_fixed_vec_iter() {
        static mut FIXED_VEC: FixedVec<i32, 10> = FixedVec::new();
        unsafe {
            FIXED_VEC.push(1);
            FIXED_VEC.push(2);
            FIXED_VEC.push(3);

            let mut iter = FIXED_VEC.iter();
            assert_eq!(iter.next(), Some(&1));
            assert_eq!(iter.next(), Some(&2));
            assert_eq!(iter.next(), Some(&3));
            assert_eq!(iter.next(), None);
            assert_eq!(iter.next(), None);

            let mut iter = FIXED_VEC.iter();
            assert_eq!(iter.next(), Some(&1));
            assert_eq!(iter.next(), Some(&2));
            assert_eq!(iter.next(), Some(&3));
            assert_eq!(iter.next(), None);

            let mut iter = FIXED_VEC.iter_mut();
            assert_eq!(iter.next(), Some(&mut 1));
            assert_eq!(iter.next(), Some(&mut 2));
            assert_eq!(iter.next(), Some(&mut 3));
            assert_eq!(iter.next(), None);

            let mut iter = FIXED_VEC.iter_mut();
            assert_eq!(iter.next(), Some(&mut 1));
            assert_eq!(iter.next(), Some(&mut 2));
            assert_eq!(iter.next(), Some(&mut 3));
            assert_eq!(iter.next(), None);

            for elem in FIXED_VEC.iter_mut() {
                *elem = *elem * 2;
            }
            let mut iter = FIXED_VEC.iter_mut();
            assert_eq!(iter.next(), Some(&mut 2));
            assert_eq!(iter.next(), Some(&mut 4));
            assert_eq!(iter.next(), Some(&mut 6));
            assert_eq!(iter.next(), None);
        }
    }
}
