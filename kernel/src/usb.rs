pub mod xhci {
    use core::mem::{size_of, MaybeUninit};
    use core::ptr::{addr_of_mut, null_mut};

    #[derive(Debug)]
    pub enum Error {
        NoEnoughMemory,
    }
    pub type Result<T> = core::result::Result<T, Error>;

    #[repr(transparent)]
    struct MemMapped<T>(T);
    impl<T> MemMapped<T> {
        pub fn read(&self) -> T {
            unsafe { (self as *const Self as *const T).read_volatile() }
        }
        pub fn write(&mut self, val: T) {
            unsafe { (self as *const Self as *mut T).write_volatile(val) }
        }
        pub fn modify_with<F: FnOnce(&mut T)>(&mut self, f: F) {
            let mut val = self.read();
            f(&mut val);
            self.write(val);
        }
    }

    macro_rules! getter {
        ($base:tt $([$idx:literal])? : $base_ty:ty ; $mask:expr, $offset:expr ; $ty:ty, $getter_name:ident) => {
            pub fn $getter_name(&self) -> $ty {
                (((self.$base $([$idx])?) & $mask) >> ($offset)) as $ty
            }
        };
    }
    macro_rules! setter {
        ($base:tt $([$idx:literal])? : $base_ty:ty ; $mask:expr, $offset:expr ; $ty:ty, $setter_name:ident) => {
            pub fn $setter_name(&mut self, val: $ty) {
                self.$base $([$idx])? = self.$base $([$idx])? & (!$mask) | ((val as $base_ty) << $offset);
            }
        };
    }

    #[allow(dead_code)]
    mod bitmap {

        #[repr(C)]
        pub struct Generic<T>(pub T);

        #[repr(C)]
        pub struct Hcsparams1 {
            data: u32,
        }
        impl Hcsparams1 {
            getter!(data: u32; 0xFF, 0; u8, max_device_slots);
        }
        #[repr(C)]
        pub struct Hcsparams2 {
            data: u32,
        }
        #[repr(C)]
        pub struct Hcsparams3 {
            data: u32,
        }
        #[repr(C)]
        pub struct Hccparams1 {
            data: u32,
        }
        impl Hccparams1 {
            getter!(data: u32; 0xFFFF0000, 16; u16, xhci_extended_capabilities_pointer);
        }

        #[repr(C)]
        pub struct Dboff {
            data: u32,
        }

        #[repr(C)]
        pub struct Rtsoff {
            data: u32,
        }
        impl Rtsoff {
            // RO
            getter!(data: u32; 0xFFFFFFE0, 5; u32, runtime_register_space_offset);

            pub fn offset(&self) -> usize {
                (self.runtime_register_space_offset() << 5) as usize
            }
        }

        #[repr(C)]
        pub struct Usbcmd {
            data: u32,
        }
        impl Usbcmd {
            getter!(data: u32; 0b0001, 0; u8, run_stop);
            setter!(data: u32; 0b0001, 0; u8, set_run_stop);

            getter!(data: u32; 0b0010, 1; u8, host_controller_reset);
            setter!(data: u32; 0b0010, 1; u8, set_host_controller_reset);

            getter!(data: u32; 0b0100, 2; u8, interrupter_enable);
            setter!(data: u32; 0b0100, 2; u8, set_interrupter_enable);
        }

        #[repr(C)]
        pub struct Usbsts {
            data: u32,
        }
        impl Usbsts {
            // RO
            getter!(data: u32; 0b0001, 0; u8, host_controller_halted);

            // RO
            getter!(data: u32; 0b100000000000, 11; u8, controller_not_ready);
        }

        #[repr(C)]
        pub struct Crcr {
            data: u64,
        }
        impl Crcr {
            getter!(data: u64; 0x0000000000000001, 0; u64, ring_cycle_state);
            setter!(data: u64; 0x0000000000000001, 0; u64, set_ring_cycle_state);

            // RW1S
            getter!(data: u64; 0x0000000000000002, 1; u64, command_stop);
            setter!(data: u64; 0x0000000000000002, 1; u64, set_command_stop);

            // RW1S
            getter!(data: u64; 0x0000000000000004, 2; u64, command_abort);
            setter!(data: u64; 0x0000000000000004, 2; u64, set_command_abort);

            getter!(data: u64; 0xFFFFFFFFFFFFFFC0, 6; u64, command_ring_pointer);
            setter!(data: u64; 0xFFFFFFFFFFFFFFC0, 6; u64, set_command_ring_pointer);

            pub fn pointer(&self) -> usize {
                (self.command_ring_pointer() << 6) as usize
            }
            pub fn set_pointer(&mut self, ptr: usize) {
                self.set_command_ring_pointer((ptr as u64) >> 6)
            }
        }

        #[derive(Default)]
        #[repr(C)]
        pub struct Dcbaap {
            data: u64,
        }
        impl Dcbaap {
            getter!(data: u64; 0xFFFFFFFFFFFFFFC0, 6; u64, device_context_base_address_array_pointer);
            setter!(data: u64; 0xFFFFFFFFFFFFFFC0, 6; u64, set_device_context_base_address_array_pointer);
            pub fn pointer(&self) -> usize {
                (self.device_context_base_address_array_pointer() << 6) as usize
            }
            pub fn set_pointer(&mut self, ptr: usize) {
                self.set_device_context_base_address_array_pointer((ptr as u64) >> 6)
            }
        }

        #[repr(C)]
        pub struct Config {
            data: u32,
        }
        impl Config {
            getter!(data: u32; 0xFF, 0; u8, max_device_slots_enabled);
            setter!(data: u32; 0xFF, 0; u8, set_max_device_slots_enabled);
        }

        #[repr(C)]
        pub struct ExtendedRegister {
            data: u32,
        }
        impl ExtendedRegister {
            // RO
            getter!(data: u32; 0x00FF, 0; u8, capability_id);
            // RO
            getter!(data: u32; 0xFF00, 8; u8, next_capability_pointer);
        }

        #[repr(C)]
        pub struct Usblegsup {
            data: u32,
        }
        impl Usblegsup {
            // RO
            getter!(data: u32; 0x00FF, 0; u8, capability_id);
            // RO
            getter!(data: u32; 0xFF00, 8; u8, next_capability_pointer);

            getter!(data: u32; 0x00010000, 16; u8, hc_bios_owned_semaphore);
            setter!(data: u32; 0x00010000, 16; u8, set_hc_bios_owned_semaphore);

            getter!(data: u32; 0x01000000, 24; u8, hc_os_owned_semaphore);
            setter!(data: u32; 0x01000000, 24; u8, set_hc_os_owned_semaphore);
        }

        #[repr(C)]
        pub struct Iman {
            data: u32,
        }
        impl Iman {
            // RW1C
            getter!(data: u32; 0x00000001, 0; u8, interrupt_pending);
            setter!(data: u32; 0x00000001, 0; u8, set_interrupt_pending);

            getter!(data: u32; 0x00000002, 1; u8, interrupter_enable);
            setter!(data: u32; 0x00000002, 1; u8, set_interrupt_enable);
        }

        #[repr(C)]
        pub struct Imod {
            data: u32,
        }

        #[repr(C)]
        pub struct Erstsz {
            data: u32,
        }
        impl Erstsz {
            getter!(data: u32; 0x0000FFFF, 0; u16, event_ring_segment_table_size);
            setter!(data: u32; 0x0000FFFF, 0; u16, set_event_ring_segment_table_size);
        }

        #[repr(C)]
        pub struct Erstba {
            data: u64,
        }
        impl Erstba {
            getter!(data: u64; 0xFFFFFFFFFFFFFFC0, 6; u64, event_ring_segment_table_base_address);
            setter!(data: u64; 0xFFFFFFFFFFFFFFC0, 6; u64, set_event_ring_segment_table_base_address);

            pub fn pointer(&self) -> usize {
                (self.event_ring_segment_table_base_address() << 6) as usize
            }
            pub fn set_pointer(&mut self, ptr: usize) {
                self.set_event_ring_segment_table_base_address((ptr as u64) >> 6)
            }
        }

        #[repr(C)]
        pub struct Erdp {
            data: u64,
        }
        impl Erdp {
            getter!(data: u64; 0xFFFFFFFFFFFFFFF0, 4; u64, event_ring_dequeue_pointer);
            setter!(data: u64; 0xFFFFFFFFFFFFFFF0, 4; u64, set_event_ring_dequeue_pointer);

            pub fn pointer(&self) -> usize {
                (self.event_ring_dequeue_pointer() << 6) as usize
            }
            pub fn set_pointer(&mut self, ptr: usize) {
                self.set_event_ring_dequeue_pointer((ptr as u64) >> 6)
            }
        }
    }

    #[repr(C)]
    struct CapabilityRegisters {
        caplength: MemMapped<bitmap::Generic<u8>>,
        _reserved1: u8,
        hciversion: MemMapped<bitmap::Generic<u16>>,
        hcsparams1: MemMapped<bitmap::Hcsparams1>,
        hcsparams2: MemMapped<bitmap::Hcsparams2>,
        hcsparams3: MemMapped<bitmap::Hcsparams3>,
        hccparams1: MemMapped<bitmap::Hccparams1>,
        dboff: MemMapped<bitmap::Dboff>,
        rtsoff: MemMapped<bitmap::Rtsoff>,
    }

    #[repr(C)]
    struct OperationalRegisters {
        usbcmd: MemMapped<bitmap::Usbcmd>,
        usbsts: MemMapped<bitmap::Usbsts>,
        pagesize: MemMapped<bitmap::Generic<u32>>,
        _reserved1: [u32; 2],
        dnctrl: MemMapped<bitmap::Generic<u32>>,
        crcr: MemMapped<bitmap::Crcr>,
        _reserved2: [u32; 4],
        dcbaap: MemMapped<bitmap::Dcbaap>,
        config: MemMapped<bitmap::Config>,
    }

    #[repr(C)]
    struct InterrupterRegisterSet {
        iman: MemMapped<bitmap::Iman>,
        imod: MemMapped<bitmap::Imod>,
        erstsz: MemMapped<bitmap::Erstsz>,
        _reserved: u32,
        erstba: MemMapped<bitmap::Erstba>,
        erdp: MemMapped<bitmap::Erdp>,
    }

    pub struct Controller {
        mmio_base: usize,
        cap_regs: *mut CapabilityRegisters,
        op_regs: *mut OperationalRegisters,
        devmgr: DeviceManager,
        cr: CommandRing,
        er: EventRing,
    }
    impl Controller {
        pub fn new(mmio_base: usize) -> Result<Self> {
            let cap_regs = mmio_base as *mut CapabilityRegisters;
            let caplength = unsafe { (*cap_regs).caplength.read() };
            let op_regs = (mmio_base + caplength.0 as usize) as *mut OperationalRegisters;
            Ok(Self {
                mmio_base,
                cap_regs,
                op_regs,
                devmgr: DeviceManager::new(),
                cr: CommandRing::with_capacity(32)?,
                er: EventRing::with_capacity(32)?,
            })
        }

        fn request_hc_ownership(&mut self) {
            type MmExtendedReg = MemMapped<bitmap::ExtendedRegister>;

            fn next(current: *mut MmExtendedReg, step: usize) -> *mut MmExtendedReg {
                if step == 0 {
                    null_mut()
                } else {
                    unsafe { current.add(step as usize) }
                }
            }

            let hccp = unsafe { (*self.cap_regs).hccparams1.read() };
            let mut ptr = next(
                self.mmio_base as *mut _,
                hccp.xhci_extended_capabilities_pointer() as usize,
            );
            let usb_reg_sup = loop {
                const USB_LEGACY_SUPPORT: u8 = 0x01;
                if unsafe { (*ptr).read().capability_id() } == USB_LEGACY_SUPPORT {
                    break Some(ptr);
                }
                let next_ptr = unsafe { (*ptr).read().next_capability_pointer() };
                ptr = next(ptr, next_ptr as usize);
                if ptr.is_null() {
                    break None;
                }
            };

            let reg = match usb_reg_sup {
                None => {
                    debug!("No USB legacy support");
                    return;
                }
                Some(reg) => reg as *mut MemMapped<bitmap::Usblegsup>,
            };

            let mut r = unsafe { (*reg).read() };
            if r.hc_os_owned_semaphore() == 1 {
                debug!("already os owned");
                return;
            }
            r.set_hc_os_owned_semaphore(1);
            unsafe { (*reg).write(r) };

            debug!("waiting until OS owns xHC...");
            loop {
                let r = unsafe { (*reg).read() };
                if r.hc_bios_owned_semaphore() == 0 && r.hc_os_owned_semaphore() == 1 {
                    break;
                }
            }
            debug!("OS has owned xHC");
        }

        fn register_command_ring(&mut self) {
            unsafe {
                (*self.op_regs).crcr.modify_with(|value| {
                    value.set_ring_cycle_state(1);
                    value.set_command_stop(0);
                    value.set_command_abort(0);
                    value.set_pointer(self.cr.buffer());
                })
            };
        }

        fn interrupter_register_sets(&self) -> *mut InterrupterRegisterSet {
            let rtsoff = unsafe { (*self.cap_regs).rtsoff.read().offset() };
            (self.mmio_base + rtsoff + 0x20) as *mut InterrupterRegisterSet
        }

        const DEVICES_SIZE: usize = 8;

        pub unsafe fn initialize(&mut self) -> Result<()> {
            // Host controller must be halted
            if (*self.op_regs).usbsts.read().host_controller_halted() == 0 {
                panic!("Host controller not halted");
            }

            self.request_hc_ownership();

            // Reset controller
            (*self.op_regs).usbcmd.modify_with(|usbcmd| {
                usbcmd.set_host_controller_reset(1);
            });
            while (*self.op_regs).usbcmd.read().host_controller_reset() != 0 {}
            while (*self.op_regs).usbsts.read().controller_not_ready() != 0 {}

            let max_slots = (*self.cap_regs).hcsparams1.read().max_device_slots();
            debug!("MaxSlots: {}", max_slots);
            assert!(Self::DEVICES_SIZE < max_slots as usize);
            let slots = core::cmp::min(max_slots, Self::DEVICES_SIZE as u8);

            self.devmgr.initialize(slots as usize)?;

            // Set "Max Slots Enabled" field in CONFIG
            (*self.op_regs).config.modify_with(|config| {
                config.set_max_device_slots_enabled(slots);
            });

            let mut dcbaap = bitmap::Dcbaap::default();
            let device_contexts = self.devmgr.device_contexts();
            dcbaap.set_pointer(device_contexts as usize);
            (*self.op_regs).dcbaap.write(dcbaap);

            let primary_interrupter = self.interrupter_register_sets();
            self.register_command_ring();
            self.er.initialize(primary_interrupter);

            // Enable interrupt for the primary interrupter
            (*primary_interrupter).iman.modify_with(|iman| {
                iman.set_interrupt_pending(1);
                iman.set_interrupt_enable(1);
            });

            // Enable interrupt for the controller
            (*self.op_regs).usbcmd.modify_with(|usbcmd| {
                usbcmd.set_interrupter_enable(1);
            });

            Ok(())
        }

        pub fn run(&mut self) {
            // Run the controller
            unsafe {
                (*self.op_regs).usbcmd.modify_with(|usbcmd| {
                    usbcmd.set_run_stop(1);
                })
            };
            while unsafe { (*self.op_regs).usbsts.read().host_controller_halted() } == 1 {}
        }
    }

    struct Ring {
        buf: *mut [Trb],
        cycle_bit: bool,
        write_idx: usize,
    }
    impl Ring {
        pub fn with_capacity(buf_size: usize) -> Result<Self> {
            let buf = unsafe { MALLOC.alloc_slice_ext::<Trb>(buf_size, 64, 64 * 1024) }
                .ok_or(Error::NoEnoughMemory)?;
            {
                let buf: &mut [_] = unsafe { &mut *buf.as_ptr() };
                for p in buf {
                    *p = MaybeUninit::zeroed();
                }
            }
            Ok(Self {
                buf: buf.as_ptr() as *mut [Trb],
                cycle_bit: true,
                write_idx: 0,
            })
        }
        pub fn buffer(&self) -> usize {
            unsafe { (*self.buf).as_ptr() as usize }
        }
    }
    type CommandRing = Ring;
    type TransferRing = Ring;

    struct EventRing {
        buf: *mut [Trb],
        erst: *mut [EventRingSegmentTableEntry],
        cycle_bit: bool,
        interrupter: *mut InterrupterRegisterSet,
    }
    impl EventRing {
        pub fn with_capacity(buf_size: usize) -> Result<Self> {
            let buf = unsafe { MALLOC.alloc_slice_ext::<Trb>(buf_size, 64, 64 * 1024) }
                .ok_or(Error::NoEnoughMemory)?;
            {
                let buf: &mut [_] = unsafe { &mut *buf.as_ptr() };
                for p in buf {
                    *p = MaybeUninit::zeroed();
                }
            }

            let table =
                unsafe { MALLOC.alloc_slice_ext::<EventRingSegmentTableEntry>(1, 64, 64 * 1024) }
                    .ok_or(Error::NoEnoughMemory)?;
            {
                let table: &mut [_] = unsafe { &mut *table.as_ptr() };
                for p in table {
                    *p = MaybeUninit::zeroed();
                }
            }
            {
                let buf = unsafe { &mut *buf.as_ptr() };
                let table: &mut [EventRingSegmentTableEntry] =
                    unsafe { &mut *(table.as_ptr() as *mut [EventRingSegmentTableEntry]) };
                table[0].set_pointer(buf.as_ptr() as usize);
                table[0].set_ring_segment_size(buf.len() as u16);
            }

            Ok(Self {
                buf: buf.as_ptr() as *mut [Trb],
                erst: table.as_ptr() as *mut [EventRingSegmentTableEntry],
                cycle_bit: true,
                interrupter: null_mut(),
            })
        }

        pub fn initialize(&mut self, interrupter: *mut InterrupterRegisterSet) {
            self.interrupter = interrupter;
            let table = self.event_ring_segment_table();
            let ptr = table.as_mut_ptr() as usize;
            let len = table.len();

            unsafe {
                (*interrupter).erstsz.modify_with(|erstsz| {
                    erstsz.set_event_ring_segment_table_size(len as u16);
                })
            };

            self.write_dequeue_pointer(unsafe { &(*self.buf)[0] });

            unsafe {
                (*interrupter).erstba.modify_with(|erstba| {
                    erstba.set_pointer(ptr);
                })
            };
        }

        pub fn write_dequeue_pointer(&mut self, ptr: *const Trb) {
            unsafe {
                (*self.interrupter).erdp.modify_with(|erdp| {
                    erdp.set_pointer(ptr as usize);
                })
            };
        }

        fn event_ring_segment_table(&mut self) -> &mut [EventRingSegmentTableEntry] {
            unsafe { &mut *self.erst }
        }
    }

    #[repr(C, align(64))]
    struct EventRingSegmentTableEntry {
        data: [u64; 2],
    }
    impl EventRingSegmentTableEntry {
        getter!(data[0]: u64; 0xFFFFFFFFFFFFFFC0, 6; u64, ring_segment_base_address);
        setter!(data[0]: u64; 0xFFFFFFFFFFFFFFC0, 6; u64, set_ring_segment_base_address);
        getter!(data[1]: u64; 0x000000000000FFFF, 0; u16, ring_segment_size);
        setter!(data[1]: u64; 0x000000000000FFFF, 0; u16, set_ring_segment_size);

        pub fn pointer(&self) -> usize {
            (self.ring_segment_base_address() << 6) as usize
        }
        pub fn set_pointer(&mut self, ptr: usize) {
            self.set_ring_segment_base_address((ptr as u64) >> 6);
        }
    }

    #[repr(C, align(64))]
    struct Trb {
        data: [u32; 4],
    }

    #[repr(C, align(64))]
    struct SlotContext {
        data: [u32; 8],
    }

    #[repr(C, align(64))]
    struct EndpointContext {
        data: [u32; 8],
    }

    #[repr(transparent)]
    struct DeviceContextIndex {
        value: usize,
    }

    #[repr(C, align(64))]
    struct DeviceContext {
        slot_context: SlotContext,
        ep_contexts: [EndpointContext; 31],
    }
    impl DeviceContext {
        pub unsafe fn initialize_ptr(ptr: *mut Self) {
            let ptr = ptr as *mut u8;
            ptr.write_bytes(0, size_of::<Self>());
        }
    }

    #[repr(C, align(64))]
    struct InputControlContext {
        drop_context_flags: u32,
        add_context_flags: u32,
        _reserved1: [u32; 5],
        configuration_value: u8,
        interface_number: u8,
        alternate_setting: u8,
        _reserved2: u8,
    }

    #[repr(C, align(64))]
    struct InputContext {
        input_control_context: InputControlContext,
        slot_context: SlotContext,
        ep_contexts: [EndpointContext; 31],
    }
    impl InputContext {
        pub unsafe fn initialize_ptr(ptr: *mut Self) {
            let ptr = ptr as *mut u8;
            ptr.write_bytes(0, size_of::<Self>());
        }
    }

    enum DeviceState {
        Invalid,
        Blank,
        SlotAssigning,
        SlotAssigned,
    }

    struct Device {
        ctx: DeviceContext,
        input_ctx: InputContext,
        state: DeviceState,
    }
    impl Device {
        pub unsafe fn initialize_ptr(ptr: *mut Self) {
            let state_ptr = addr_of_mut!((*ptr).state);
            state_ptr.write(DeviceState::Blank);

            let ctx_ptr = addr_of_mut!((*ptr).ctx);
            DeviceContext::initialize_ptr(ctx_ptr);

            let input_ctx_ptr = addr_of_mut!((*ptr).input_ctx);
            InputContext::initialize_ptr(input_ctx_ptr);
        }
    }

    struct DeviceManager {
        devices: *mut [Device],
        device_context_pointers: *mut [*mut DeviceContext],
        num_devices: usize,
    }
    impl DeviceManager {
        pub fn new() -> Self {
            use core::ptr::slice_from_raw_parts_mut;
            Self {
                devices: slice_from_raw_parts_mut(null_mut(), 0),
                device_context_pointers: slice_from_raw_parts_mut(null_mut(), 0),
                num_devices: 0,
            }
        }
        pub fn initialize(&mut self, num_devices: usize) -> Result<()> {
            self.num_devices = num_devices;

            let devices = unsafe { MALLOC.alloc_slice::<Device>(num_devices) }
                .ok_or(Error::NoEnoughMemory)?;
            {
                let devices: &mut [MaybeUninit<Device>] = unsafe { &mut *devices.as_ptr() };
                for p in devices.iter_mut() {
                    unsafe { Device::initialize_ptr(p.as_mut_ptr()) };
                }
            }
            self.devices = devices.as_ptr() as *mut [Device];

            let device_context_pointers =
                unsafe { MALLOC.alloc_slice::<*mut DeviceContext>(num_devices + 1) }
                    .ok_or(Error::NoEnoughMemory)?;
            {
                let device_context_pointers: &mut [MaybeUninit<*mut DeviceContext>] =
                    unsafe { &mut *device_context_pointers.as_ptr() };
                for p in device_context_pointers.iter_mut() {
                    *p = MaybeUninit::new(null_mut());
                }
            }
            self.device_context_pointers =
                device_context_pointers.as_ptr() as *mut [*mut DeviceContext];

            debug!(
                "DeviceManager has been initialized for up to {} devices",
                num_devices
            );

            Ok(())
        }

        pub fn device_contexts(&self) -> *mut *mut DeviceContext {
            let ptr = unsafe { (*self.device_context_pointers).as_mut_ptr() };
            assert!((ptr as usize) % 64 == 0);
            ptr
        }
    }

    use crate::utils::StaticMallocator;
    const BUF_SIZE: usize = 4096 * 32;
    static mut MALLOC: StaticMallocator<BUF_SIZE> = StaticMallocator::new();
}
