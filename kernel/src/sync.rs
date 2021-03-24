pub mod spin {
    use core::hint::spin_loop;
    use core::sync::atomic::{fence, AtomicBool, Ordering};

    pub struct SpinLock {
        locked: AtomicBool,
        // for debugging
        _name: &'static str,
    }

    impl SpinLock {
        pub const fn new(name: &'static str) -> Self {
            Self {
                locked: AtomicBool::new(false),
                _name: name,
            }
        }

        /// Acquire the lock.
        /// Loops (spins) until the lock is acquired.
        /// Holding a lock for a long time may cause
        /// other CPUs to waste time spinning to acquire it.
        pub fn acquire(&self) {
            // TODO: stop interrupttion

            while self
                .locked
                .compare_exchange(false, true, Ordering::Relaxed, Ordering::Relaxed)
                .is_err()
            {
                spin_loop();
            }

            // Tell the compiler and the processor to not move loads or stores
            // past this point, to ensure that the critical section's memory
            // references happen after the lock is acquired.
            fence(Ordering::Acquire);
        }

        /// Release the lock.
        pub fn release(&self) {
            // Tell the compiler and the processor to not move loads or stores
            // past this point, to ensure that all the stores in the critical
            // section are visible to other cores before the lock is released.
            fence(Ordering::Release);

            // Release the lock
            self.locked.store(false, Ordering::Relaxed);

            // TODO: enable interrupttion
        }
    }

    use core::cell::UnsafeCell;
    pub struct SpinMutex<T> {
        lock: SpinLock,
        data: UnsafeCell<T>,
    }
    impl<T> SpinMutex<T> {
        pub const fn new(name: &'static str, data: T) -> Self {
            Self {
                lock: SpinLock::new(name),
                data: UnsafeCell::new(data),
            }
        }
        pub fn lock(&self) -> SpinMutexGuard<'_, T> {
            self.lock.acquire();
            SpinMutexGuard { mtx: self }
        }
    }
    unsafe impl<T: Send> Send for SpinMutex<T> {}
    unsafe impl<T: Send> Sync for SpinMutex<T> {}

    pub struct SpinMutexGuard<'a, T: 'a> {
        mtx: &'a SpinMutex<T>,
    }

    impl<'a, T: 'a> Drop for SpinMutexGuard<'a, T> {
        fn drop(&mut self) {
            self.mtx.lock.release();
        }
    }

    use core::ops::{Deref, DerefMut};
    impl<'a, T: 'a> Deref for SpinMutexGuard<'a, T> {
        type Target = T;
        fn deref(&self) -> &Self::Target {
            unsafe { &*self.mtx.data.get() }
        }
    }
    impl<'a, T: 'a> DerefMut for SpinMutexGuard<'a, T> {
        fn deref_mut(&mut self) -> &mut Self::Target {
            unsafe { &mut *self.mtx.data.get() }
        }
    }

    #[allow(dead_code)]
    impl<'a, T: 'a> SpinMutexGuard<'a, T> {
        pub unsafe fn force_unlock(&self) {
            self.mtx.lock.release()
        }
        pub unsafe fn force_lock(&self) {
            self.mtx.lock.release()
        }
    }
}
