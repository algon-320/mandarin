pub mod xhci {
    use core::mem::{size_of, MaybeUninit};
    use core::ptr::{addr_of_mut, null_mut};

    #[derive(Debug)]
    pub enum Error {
        NoEnoughMemory,
        InvalidPhase,
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

    #[allow(dead_code)]
    mod bitmap {
        #[repr(C)]
        pub struct Hcsparams1 {
            data: u32,
        }
        impl Hcsparams1 {
            getter!(data: u32; 0x000000FF; u8, pub max_device_slots);

            getter!(data: u32; 0xFF000000; u8, pub max_ports);
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
            getter!(data: u32; 0xFFFF0000; u16, pub xhci_extended_capabilities_pointer);
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
            getter!(data: u32; 0xFFFFFFE0; u32, runtime_register_space_offset);

            pub fn offset(&self) -> usize {
                (self.runtime_register_space_offset() << 5) as usize
            }
        }

        #[repr(C)]
        pub struct Usbcmd {
            data: u32,
        }
        impl Usbcmd {
            getter!(data: u32; 0b0001; u8, pub run_stop);
            setter!(data: u32; 0b0001; u8, pub set_run_stop);

            getter!(data: u32; 0b0010; u8, pub host_controller_reset);
            setter!(data: u32; 0b0010; u8, pub set_host_controller_reset);

            getter!(data: u32; 0b0100; u8, pub interrupter_enable);
            setter!(data: u32; 0b0100; u8, pub set_interrupter_enable);
        }

        #[repr(C)]
        pub struct Usbsts {
            data: u32,
        }
        impl Usbsts {
            // RO
            getter!(data: u32; 0x00000001; u8, pub host_controller_halted);

            // RO
            getter!(data: u32; 0x00000800; u8, pub controller_not_ready);
        }

        #[repr(C)]
        pub struct Crcr {
            data: u64,
        }
        impl Crcr {
            getter!(data: u64; 0x0000000000000001; u64, pub ring_cycle_state);
            setter!(data: u64; 0x0000000000000001; u64, pub set_ring_cycle_state);

            // RW1S
            getter!(data: u64; 0x0000000000000002; u64, pub command_stop);
            setter!(data: u64; 0x0000000000000002; u64, pub set_command_stop);

            // RW1S
            getter!(data: u64; 0x0000000000000004; u64, pub command_abort);
            setter!(data: u64; 0x0000000000000004; u64, pub set_command_abort);

            getter!(data: u64; 0xFFFFFFFFFFFFFFC0; u64, command_ring_pointer);
            setter!(data: u64; 0xFFFFFFFFFFFFFFC0; u64, set_command_ring_pointer);

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
            getter!(data: u64; 0xFFFFFFFFFFFFFFC0; u64, device_context_base_address_array_pointer);
            setter!(data: u64; 0xFFFFFFFFFFFFFFC0; u64, set_device_context_base_address_array_pointer);
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
            getter!(data: u32; 0xFF; u8, pub max_device_slots_enabled);
            setter!(data: u32; 0xFF; u8, pub set_max_device_slots_enabled);
        }

        #[repr(C)]
        pub struct ExtendedRegister {
            data: u32,
        }
        impl ExtendedRegister {
            // RO
            getter!(data: u32; 0x00FF; u8, pub capability_id);
            // RO
            getter!(data: u32; 0xFF00; u8, pub next_capability_pointer);
        }

        #[repr(C)]
        pub struct Usblegsup {
            data: u32,
        }
        impl Usblegsup {
            // RO
            getter!(data: u32; 0x000000FF; u8, pub capability_id);
            // RO
            getter!(data: u32; 0x0000FF00; u8, pub next_capability_pointer);

            getter!(data: u32; 0x00010000; u8, pub hc_bios_owned_semaphore);
            setter!(data: u32; 0x00010000; u8, pub set_hc_bios_owned_semaphore);

            getter!(data: u32; 0x01000000; u8, pub hc_os_owned_semaphore);
            setter!(data: u32; 0x01000000; u8, pub set_hc_os_owned_semaphore);
        }

        #[repr(C)]
        pub struct Iman {
            data: u32,
        }
        impl Iman {
            // RW1C
            getter!(data: u32; 0x00000001; u8, pub interrupt_pending);
            setter!(data: u32; 0x00000001; u8, pub set_interrupt_pending);

            getter!(data: u32; 0x00000002; u8, pub interrupter_enable);
            setter!(data: u32; 0x00000002; u8, pub set_interrupt_enable);
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
            getter!(data: u32; 0x0000FFFF; u16, pub event_ring_segment_table_size);
            setter!(data: u32; 0x0000FFFF; u16, pub set_event_ring_segment_table_size);
        }

        #[repr(C)]
        pub struct Erstba {
            data: u64,
        }
        impl Erstba {
            getter!(data: u64; 0xFFFFFFFFFFFFFFC0; u64, event_ring_segment_table_base_address);
            setter!(data: u64; 0xFFFFFFFFFFFFFFC0; u64, set_event_ring_segment_table_base_address);

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
            getter!(data: u64; 0xFFFFFFFFFFFFFFF0; u64, event_ring_dequeue_pointer);
            setter!(data: u64; 0xFFFFFFFFFFFFFFF0; u64, set_event_ring_dequeue_pointer);

            pub fn pointer(&self) -> usize {
                (self.event_ring_dequeue_pointer() << 4) as usize
            }
            pub fn set_pointer(&mut self, ptr: usize) {
                self.set_event_ring_dequeue_pointer((ptr as u64) >> 4)
            }
        }

        #[repr(C)]
        pub struct Portsc {
            pub data: u32,
        }
        impl Portsc {
            // ROS
            getter!(data: u32; 0x00000001; u8, pub current_connect_status);

            // RW1CS
            getter!(data: u32; 0x00000002; u8, pub port_enabled_disabled);

            // RW1S
            getter!(data: u32; 0x00000010; u8, pub port_reset);

            // ROS
            getter!(data: u32; 0x00003C00; u8, pub port_speed);

            // RW1CS
            getter!(data: u32; 0x00020000; u8, pub connect_status_change);

            // RW1CS
            getter!(data: u32; 0x00200000; u8, pub port_reset_change);
        }

        #[repr(C)]
        pub struct Portmsc {
            data: u32,
        }

        #[repr(C)]
        pub struct Portli {
            data: u32,
        }

        #[repr(C)]
        pub struct Porthlpmc {
            data: u32,
        }
    }

    #[repr(C)]
    struct CapabilityRegisters {
        caplength: MemMapped<u8>,
        _reserved1: u8,
        hciversion: MemMapped<u16>,
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
        pagesize: MemMapped<u32>,
        _reserved1: [u32; 2],
        dnctrl: MemMapped<u32>,
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

    #[repr(C)]
    struct PortRegisterSet {
        portsc: MemMapped<bitmap::Portsc>,
        portmsc: MemMapped<bitmap::Portmsc>,
        portli: MemMapped<bitmap::Portli>,
        porthlpmc: MemMapped<bitmap::Porthlpmc>,
    }

    pub struct Controller {
        mmio_base: usize,
        cap_regs: *mut CapabilityRegisters,
        op_regs: *mut OperationalRegisters,
        devmgr: DeviceManager,
        cr: CommandRing,
        er: EventRing,
        ports: *mut [Port],
        max_ports: u8,
        addressing_port: Option<u8>,
    }
    impl Controller {
        const DEVICES_SIZE: usize = 8;

        pub fn new(mmio_base: usize) -> Result<Self> {
            let cap_regs = mmio_base as *mut CapabilityRegisters;
            let caplength = unsafe { (*cap_regs).caplength.read() };
            let op_regs = (mmio_base + caplength as usize) as *mut OperationalRegisters;
            let max_ports = unsafe { (*cap_regs).hcsparams1.read().max_ports() };
            let mut er = EventRing::with_capacity(32)?;

            // Host controller must be halted
            if unsafe { (*op_regs).usbsts.read().host_controller_halted() == 0 } {
                panic!("Host controller not halted");
            }

            Self::request_hc_ownership(mmio_base, cap_regs);

            // Reset controller
            unsafe {
                (*op_regs).usbcmd.modify_with(|usbcmd| {
                    usbcmd.set_host_controller_reset(1);
                });
                while (*op_regs).usbcmd.read().host_controller_reset() != 0 {}
                while (*op_regs).usbsts.read().controller_not_ready() != 0 {}
            }

            let max_slots = unsafe { (*cap_regs).hcsparams1.read().max_device_slots() };
            debug!("MaxSlots: {}", max_slots);
            assert!(Self::DEVICES_SIZE < max_slots as usize);
            let slots = core::cmp::min(max_slots, Self::DEVICES_SIZE as u8);

            let devmgr = DeviceManager::new(slots as usize)?;

            // Set "Max Slots Enabled" field in CONFIG
            unsafe {
                (*op_regs).config.modify_with(|config| {
                    config.set_max_device_slots_enabled(slots);
                })
            };

            // TODO: scratchpad buffers

            let mut dcbaap = bitmap::Dcbaap::default();
            let device_contexts = devmgr.device_contexts();
            dcbaap.set_pointer(device_contexts as usize);
            unsafe { (*op_regs).dcbaap.write(dcbaap) };

            let primary_interrupter = Self::interrupter_register_sets(mmio_base, cap_regs);
            debug!("primary_interrupter = {:p}", primary_interrupter);

            let cr = CommandRing::with_capacity(32)?;
            // register command ring
            unsafe {
                (*op_regs).crcr.modify_with(|value| {
                    value.set_ring_cycle_state(1);
                    value.set_command_stop(0);
                    value.set_command_abort(0);
                    value.set_pointer(cr.buffer());
                })
            };

            er.initialize(primary_interrupter);

            // Enable interrupt for the primary interrupter
            unsafe {
                (*primary_interrupter).iman.modify_with(|iman| {
                    iman.set_interrupt_pending(1);
                    iman.set_interrupt_enable(1);
                })
            };

            // Enable interrupt for the controller
            unsafe {
                (*op_regs).usbcmd.modify_with(|usbcmd| {
                    usbcmd.set_interrupter_enable(1);
                })
            };

            // initialize ports
            let ports = unsafe { MALLOC.alloc_slice::<Port>(max_ports as usize) }
                .ok_or(Error::NoEnoughMemory)?;
            {
                let port_regs_origin = Self::port_register_set(op_regs);
                let ports: &mut [MaybeUninit<Port>] = unsafe { &mut *ports.as_ptr() };
                for port_num in 1..=max_ports {
                    let port_regs = unsafe { port_regs_origin.add((port_num - 1) as usize) };
                    ports[(port_num - 1) as usize] =
                        MaybeUninit::new(Port::new(port_num, port_regs));
                }
            }

            Ok(Self {
                mmio_base,
                cap_regs,
                op_regs,
                devmgr,
                cr,
                er,
                ports: ports.as_ptr() as *mut [Port],
                max_ports,
                addressing_port: None,
            })
        }

        fn request_hc_ownership(mmio_base: usize, cap_regs: *mut CapabilityRegisters) {
            type MmExtendedReg = MemMapped<bitmap::ExtendedRegister>;

            fn next(current: *mut MmExtendedReg, step: usize) -> *mut MmExtendedReg {
                if step == 0 {
                    null_mut()
                } else {
                    unsafe { current.add(step as usize) }
                }
            }

            let hccp = unsafe { (*cap_regs).hccparams1.read() };
            let mut ptr = next(
                mmio_base as *mut _,
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

        fn interrupter_register_sets(
            mmio_base: usize,
            cap_regs: *mut CapabilityRegisters,
        ) -> *mut InterrupterRegisterSet {
            let rtsoff = unsafe { (*cap_regs).rtsoff.read().offset() };
            (mmio_base + rtsoff + 0x20) as *mut InterrupterRegisterSet
        }

        fn port_register_set(op_regs: *mut OperationalRegisters) -> *mut PortRegisterSet {
            ((op_regs as usize) + 0x400) as *mut PortRegisterSet
        }

        pub fn run(&mut self) {
            info!("xHC staring");
            // Run the controller
            unsafe {
                (*self.op_regs).usbcmd.modify_with(|usbcmd| {
                    usbcmd.set_run_stop(1);
                })
            };
            while unsafe { (*self.op_regs).usbsts.read().host_controller_halted() } == 1 {}
        }

        pub fn max_ports(&self) -> u8 {
            self.max_ports
        }

        pub fn port_at(&self, port_num: u8) -> &Port {
            assert!(0 < port_num && port_num <= self.max_ports);
            unsafe { &(*self.ports)[(port_num as usize) - 1] }
        }
        pub fn port_at_mut(&mut self, port_num: u8) -> &mut Port {
            assert!(0 < port_num && port_num <= self.max_ports);
            unsafe { &mut (*self.ports)[(port_num as usize) - 1] }
        }

        pub fn configure_ports(&mut self) {
            trace!("configure_ports");
            for port_num in 1..=self.max_ports {
                if !self.port_at(port_num).is_connected() {
                    continue;
                }
                debug!("Port {}: connected", port_num);
                if self.port_at(port_num).config_phase() == PortConfigPhase::NotConnected {
                    if let Err(e) = self.reset_port(port_num) {
                        error!("Failed to configure the port {}: {:?}", port_num, e);
                    }
                }
            }
        }

        fn reset_port(&mut self, port_num: u8) -> Result<()> {
            if !self.port_at(port_num).is_connected() {
                return Ok(());
            }

            match self.addressing_port {
                Some(_) => {
                    self.port_at_mut(port_num)
                        .set_config_phase(PortConfigPhase::WaitingAddressed);
                }
                None => {
                    let port = self.port_at_mut(port_num);
                    if port.config_phase() != PortConfigPhase::NotConnected
                        && port.config_phase() != PortConfigPhase::WaitingAddressed
                    {
                        return Err(Error::InvalidPhase);
                    }
                    port.set_config_phase(PortConfigPhase::ResettingPort);
                    port.reset();
                }
            }
            Ok(())
        }

        pub fn process_event(&mut self) -> Result<()> {
            if let Some(trb) = self.er.front() {
                trace!("event found");
            }
            Ok(())
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

        pub fn front(&self) -> Option<*const Trb> {
            if self.has_front() {
                Some(self.read_dequeue_pointer())
            } else {
                None
            }
        }

        pub fn pop(&mut self) {
            let mut new_front = unsafe { self.read_dequeue_pointer().add(1) };

            // FIXME: we should consider multiple segments.
            {
                let begin = self.event_ring_segment_table()[0].pointer() as *const Trb;
                let size = self.event_ring_segment_table()[0].ring_segment_size();
                let end = unsafe { begin.add(size as usize) };

                if new_front == end {
                    new_front = begin;
                    self.cycle_bit = !self.cycle_bit;
                }
            }

            self.write_dequeue_pointer(new_front);
        }

        fn write_dequeue_pointer(&mut self, ptr: *const Trb) {
            unsafe {
                (*self.interrupter).erdp.modify_with(|erdp| {
                    erdp.set_pointer(ptr as usize);
                })
            };
        }

        fn read_dequeue_pointer(&self) -> *const Trb {
            unsafe { (*self.interrupter).erdp.read().pointer() as *const Trb }
        }

        fn has_front(&self) -> bool {
            unsafe { (*self.read_dequeue_pointer()).cycle_bit() == self.cycle_bit as u8 }
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
        getter!(data[0]: u64; 0xFFFFFFFFFFFFFFC0; u64, ring_segment_base_address);
        setter!(data[0]: u64; 0xFFFFFFFFFFFFFFC0; u64, set_ring_segment_base_address);

        getter!(data[1]: u64; 0x000000000000FFFF; u16, pub ring_segment_size);
        setter!(data[1]: u64; 0x000000000000FFFF; u16, pub set_ring_segment_size);

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
    impl Trb {
        getter!(data[2]: u32; 0xFFFFFFFF; u32, pub status);
        setter!(data[2]: u32; 0xFFFFFFFF; u32, pub set_status);

        getter!(data[3]: u32; 0x00000001;  u8, pub cycle_bit);
        setter!(data[3]: u32; 0x00000001;  u8, pub set_cycle_bit);

        getter!(data[3]: u32; 0x00000002;  u8, pub evaluate_next_trb);
        setter!(data[3]: u32; 0x00000002;  u8, pub set_evaluate_next_trb);

        getter!(data[3]: u32; 0x0000FC00;  u8, pub trb_type);
        setter!(data[3]: u32; 0x0000FC00;  u8, pub set_trb_type);

        getter!(data[3]: u32; 0xFFFF0000; u16, pub control);
        setter!(data[3]: u32; 0xFFFF0000; u16, pub set_control);
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
        pub fn new(num_devices: usize) -> Result<Self> {
            let devices = unsafe { MALLOC.alloc_slice::<Device>(num_devices) }
                .ok_or(Error::NoEnoughMemory)?;
            {
                let devices: &mut [MaybeUninit<Device>] = unsafe { &mut *devices.as_ptr() };
                for p in devices.iter_mut() {
                    unsafe { Device::initialize_ptr(p.as_mut_ptr()) };
                }
            }

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

            debug!(
                "DeviceManager has been initialized for up to {} devices",
                num_devices
            );

            Ok(Self {
                devices: devices.as_ptr() as *mut [Device],
                device_context_pointers: device_context_pointers.as_ptr()
                    as *mut [*mut DeviceContext],
                num_devices,
            })
        }

        pub fn device_contexts(&self) -> *mut *mut DeviceContext {
            let ptr = unsafe { (*self.device_context_pointers).as_mut_ptr() };
            assert!((ptr as usize) % 64 == 0);
            ptr
        }
    }

    pub enum PortSpeed {
        Full,
        Low,
        High,
        Super,
        SuperPlus,
    }
    impl From<u8> for PortSpeed {
        fn from(speed: u8) -> Self {
            match speed {
                1 => Self::Full,
                2 => Self::Low,
                3 => Self::High,
                4 => Self::Super,
                5 => Self::SuperPlus,
                _ => panic!("invalid speed: {}", speed),
            }
        }
    }

    #[derive(Clone, Copy, PartialEq, Eq)]
    pub enum PortConfigPhase {
        NotConnected,
        WaitingAddressed,
        ResettingPort,
        EnablingSlot,
        AddressingDevice,
        InitializingDevice,
        ConfiguringEndpoints,
        Configured,
    }

    pub struct Port {
        port_num: u8,
        regs: *mut PortRegisterSet,
        config_phase: PortConfigPhase,
    }
    impl Port {
        fn new(port_num: u8, regs: *mut PortRegisterSet) -> Self {
            Self {
                port_num,
                regs,
                config_phase: PortConfigPhase::NotConnected,
            }
        }

        pub fn config_phase(&self) -> PortConfigPhase {
            self.config_phase
        }
        fn set_config_phase(&mut self, cp: PortConfigPhase) {
            self.config_phase = cp;
        }

        pub fn reset(&mut self) {
            unsafe {
                (*self.regs).portsc.modify_with(|portsc| {
                    portsc.data &= 0x0E00C3E0;
                    portsc.data |= 0x00020010; // Write 1 to PR and CSC
                })
            };
            while unsafe { (*self.regs).portsc.read().port_reset() == 1 } {}
        }

        pub fn number(&self) -> u8 {
            self.port_num
        }

        pub fn is_connected(&self) -> bool {
            unsafe { (*self.regs).portsc.read().current_connect_status() == 1 }
        }

        pub fn is_enabled(&self) -> bool {
            unsafe { (*self.regs).portsc.read().port_enabled_disabled() == 1 }
        }

        pub fn is_connect_status_changed(&self) -> bool {
            unsafe { (*self.regs).portsc.read().connect_status_change() == 1 }
        }

        pub fn is_port_reset_changed(&self) -> bool {
            unsafe { (*self.regs).portsc.read().port_reset_change() == 1 }
        }

        pub fn speed(&self) -> PortSpeed {
            unsafe { (*self.regs).portsc.read().port_speed() }.into()
        }
    }

    use crate::utils::StaticMallocator;
    const BUF_SIZE: usize = 4096 * 32;
    static mut MALLOC: StaticMallocator<BUF_SIZE> = StaticMallocator::new();
}

pub mod class_driver {
    pub trait Driver {}
    pub trait HidDriver: Driver {}

    pub struct HidMouseDriver {}
    impl Driver for HidMouseDriver {}
    impl HidDriver for HidMouseDriver {}

    pub struct HidKeyboardDriver {}
    impl Driver for HidKeyboardDriver {}
    impl HidDriver for HidKeyboardDriver {}
}
