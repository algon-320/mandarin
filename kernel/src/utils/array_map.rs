#![allow(dead_code)]

pub enum ArrayMapError {
    NoSpace,
    SameKeyRegistered,
}

pub struct ArrayMap<K, V, const SIZE: usize>
where
    K: PartialEq,
{
    table: [Option<(K, V)>; SIZE],
}

impl<K, V, const SIZE: usize> ArrayMap<K, V, SIZE>
where
    K: PartialEq,
{
    pub unsafe fn initialize_ptr(ptr: *mut Self) {
        let ptr = core::ptr::addr_of_mut!((*ptr).table);
        for i in 0..SIZE {
            core::ptr::addr_of_mut!((*ptr)[i]).write(None);
        }
    }

    pub fn insert(&mut self, key: K, value: V) -> Result<Option<(K, V)>, ArrayMapError> {
        for elem in self.table.iter_mut() {
            if let Some((k, _)) = elem.as_ref() {
                if k == &key {
                    return Err(ArrayMapError::SameKeyRegistered);
                }
            }
            if elem.is_none() {
                return Ok(core::mem::replace(elem, Some((key, value))));
            }
        }
        Err(ArrayMapError::NoSpace)
    }

    pub fn get(&self, key: &K) -> Option<&V> {
        for elem in self.table.iter() {
            if let Some((k, v)) = elem.as_ref() {
                if k == key {
                    return Some(v);
                }
            }
        }
        None
    }

    pub fn get_mut(&mut self, key: &K) -> Option<&mut V> {
        for elem in self.table.iter_mut() {
            if let Some((k, v)) = elem.as_mut() {
                if k == key {
                    return Some(v);
                }
            }
        }
        None
    }

    pub fn remove(&mut self, key: &K) -> Option<(K, V)> {
        for elem in self.table.iter_mut() {
            if let Some((k, _)) = elem.as_ref() {
                if k == key {
                    return elem.take();
                }
            }
        }
        None
    }
}
