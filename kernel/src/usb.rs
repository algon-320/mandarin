#[allow(clippy::transmute_ptr_to_ptr)]

pub mod xhci {
    use core::mem::{size_of, MaybeUninit};
    use core::ptr::{addr_of, addr_of_mut, null, null_mut};

    #[derive(Debug)]
    pub enum Error {
        NoEnoughMemory,
        InvalidPhase,
        InvalidSlotId,
        DeviceAlreadyAllocated,
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
        impl Dboff {
            // RO
            getter!(data: u32; 0xFFFFFFFC; u32, doorbell_array_offset);

            pub fn offset(&self) -> usize {
                (self.doorbell_array_offset() << 2) as usize
            }
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
        pub struct Pagesize {
            data: u32,
        }
        impl Pagesize {
            // RO
            getter!(data: u32; 0x0000FFFF; u32, page_size_shifted);
            pub fn page_size(&self) -> usize {
                1 << (self.page_size_shifted() as usize + 12)
            }
        }

        #[repr(C)]
        pub struct Crcr {
            data: u64,
        }
        impl Crcr {
            getter!(data: u64; 0x0000000000000001; u8, pub ring_cycle_state);
            setter!(data: u64; 0x0000000000000001; u8, pub set_ring_cycle_state);

            // RW1S
            getter!(data: u64; 0x0000000000000002; u8, pub command_stop);
            setter!(data: u64; 0x0000000000000002; u8, pub set_command_stop);

            // RW1S
            getter!(data: u64; 0x0000000000000004; u8, pub command_abort);
            setter!(data: u64; 0x0000000000000004; u8, pub set_command_abort);

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
            pub fn id() -> u8 {
                1
            }

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
            setter!(data: u32; 0x00020000; u8, pub set_connect_status_change);

            // RW1CS
            getter!(data: u32; 0x00200000; u8, pub port_reset_change);
            setter!(data: u32; 0x00200000; u8, pub set_port_reset_change);
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

        #[repr(C)]
        pub struct Doorbell {
            data: u32,
        }
        impl Doorbell {
            setter!(data: u32; 0x000000FF;  u8, pub set_db_target);
            setter!(data: u32; 0xFFFF0000; u16, pub set_db_stream_id);
        }
    }

    #[repr(C, packed(4))]
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

    #[repr(C, packed(4))]
    struct OperationalRegisters {
        usbcmd: MemMapped<bitmap::Usbcmd>,
        usbsts: MemMapped<bitmap::Usbsts>,
        pagesize: MemMapped<bitmap::Pagesize>,
        _reserved1: [u32; 2],
        dnctrl: MemMapped<u32>,
        crcr: MemMapped<bitmap::Crcr>,
        _reserved2: [u32; 4],
        dcbaap: MemMapped<bitmap::Dcbaap>,
        config: MemMapped<bitmap::Config>,
    }

    #[repr(C, packed(4))]
    struct InterrupterRegisterSet {
        iman: MemMapped<bitmap::Iman>,
        imod: MemMapped<bitmap::Imod>,
        erstsz: MemMapped<bitmap::Erstsz>,
        _reserved: u32,
        erstba: MemMapped<bitmap::Erstba>,
        erdp: MemMapped<bitmap::Erdp>,
    }

    #[repr(C, packed(4))]
    struct PortRegisterSet {
        portsc: MemMapped<bitmap::Portsc>,
        portmsc: MemMapped<bitmap::Portmsc>,
        portli: MemMapped<bitmap::Portli>,
        porthlpmc: MemMapped<bitmap::Porthlpmc>,
    }

    #[repr(C, packed(4))]
    struct DoorbellRegister {
        reg: MemMapped<bitmap::Doorbell>,
    }
    impl DoorbellRegister {
        pub fn ring(&mut self, target: u8, stream_id: u16) {
            self.reg.modify_with(|reg| {
                reg.set_db_target(target);
                reg.set_db_stream_id(stream_id);
            })
        }
    }

    pub struct Controller {
        mmio_base: usize,
        cap_regs: *mut CapabilityRegisters,
        op_regs: *mut OperationalRegisters,
        devmgr: DeviceManager,
        cr: CommandRing,
        er: Option<EventRing>,
        ports: &'static mut [Port],
        max_ports: u8,
        addressing_port: Option<u8>,
        doorbell_zero: *mut DoorbellRegister,
    }
    impl Controller {
        const DEVICES_SIZE: usize = 8;

        pub fn new(mmio_base: usize) -> Result<Self> {
            let cap_regs = mmio_base as *mut CapabilityRegisters;
            let max_ports = unsafe { (*cap_regs).hcsparams1.read().max_ports() };

            let dboff = unsafe { (*cap_regs).dboff.read().offset() };
            let doorbell_zero = (mmio_base + dboff) as *mut DoorbellRegister;

            let rtsoff = unsafe { (*cap_regs).rtsoff.read().offset() };
            let primary_interrupter = (mmio_base + rtsoff + 0x20) as *mut InterrupterRegisterSet;

            let caplength = unsafe { (*cap_regs).caplength.read() };
            let op_regs = (mmio_base + caplength as usize) as *mut OperationalRegisters;

            // Host controller must be halted
            if unsafe { (*op_regs).usbsts.read().host_controller_halted() == 0 } {
                panic!("Host controller not halted");
            }

            // Update the StaticMallocator's boundary to PAGESIZE
            let page_size = unsafe { (*op_regs).pagesize.read().page_size() };
            MALLOC.lock().default_boundary = page_size;

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

            let devmgr = DeviceManager::new(slots as usize, unsafe { doorbell_zero.add(1) })?;

            // Set "Max Slots Enabled" field in CONFIG
            unsafe {
                (*op_regs).config.modify_with(|config| {
                    config.set_max_device_slots_enabled(slots);
                })
            };

            // TODO: scratchpad buffers

            let mut dcbaap = bitmap::Dcbaap::default();
            let device_contexts = devmgr.dcbaap();
            dcbaap.set_pointer(device_contexts as usize);
            unsafe { (*op_regs).dcbaap.write(dcbaap) };

            let cr = CommandRing::with_capacity(32)?;
            // register command ring
            unsafe {
                (*op_regs).crcr.modify_with(|value| {
                    value.set_ring_cycle_state(cr.cycle_bit as u8);
                    value.set_command_stop(0);
                    value.set_command_abort(0);
                    value.set_pointer(cr.buffer_ptr() as usize);
                })
            };

            let mut er = EventRing::with_capacity(32)?;
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
            let ports = {
                let ports = MALLOC
                    .lock()
                    .alloc_slice::<Port>((max_ports + 1) as usize)
                    .ok_or(Error::NoEnoughMemory)?;
                let port_regs_origin = ((op_regs as usize) + 0x400) as *mut PortRegisterSet;
                let ports: &mut [MaybeUninit<Port>] = unsafe { &mut *ports.as_ptr() };
                for port_num in 1..=max_ports {
                    let port_regs = unsafe { port_regs_origin.add((port_num - 1) as usize) };
                    ports[port_num as usize] = MaybeUninit::new(Port::new(port_num, port_regs));
                }
                unsafe { core::mem::transmute::<&mut [MaybeUninit<Port>], &mut [Port]>(ports) }
            };

            Ok(Self {
                mmio_base,
                cap_regs,
                op_regs,
                devmgr,
                cr,
                er: Some(er),
                ports,
                max_ports,
                addressing_port: None,
                doorbell_zero,
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
            let usb_leg_sup = loop {
                if unsafe { (*ptr).read().capability_id() } == bitmap::Usblegsup::id() {
                    break Some(ptr);
                }
                let next_ptr = unsafe { (*ptr).read().next_capability_pointer() };
                ptr = next(ptr, next_ptr as usize);
                if ptr.is_null() {
                    break None;
                }
            };

            let reg = match usb_leg_sup {
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

        pub fn configure_ports(&mut self) {
            trace!("configure_ports");
            for port_num in 1..=self.max_ports {
                if !self.ports[port_num as usize].is_connected() {
                    continue;
                }
                debug!("Port {}: connected", port_num);
                if self.ports[port_num as usize].config_phase() == PortConfigPhase::NotConnected {
                    if let Err(e) = self.reset_port(port_num) {
                        error!("Failed to configure the port {}: {:?}", port_num, e);
                    }
                }
            }
        }

        fn reset_port(&mut self, port_num: u8) -> Result<()> {
            if !self.ports[port_num as usize].is_connected() {
                return Ok(());
            }

            match self.addressing_port {
                Some(_) => {
                    self.ports[port_num as usize]
                        .set_config_phase(PortConfigPhase::WaitingAddressed);
                }
                None => {
                    self.addressing_port = Some(port_num);
                    let port = &mut self.ports[port_num as usize];
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

        fn enable_slot(&mut self, port_num: u8) -> Result<()> {
            let port = &mut self.ports[port_num as usize];
            let is_enabled = port.is_enabled();
            let reset_completed = port.is_port_reset_changed();
            trace!(
                "enable_slot: is_enabled = {:?}, is_port_reset_changed = {:?}",
                is_enabled,
                reset_completed
            );
            if is_enabled && reset_completed {
                port.clear_port_reset_change();
                port.set_config_phase(PortConfigPhase::EnablingSlot);
                let cmd = trb::EnableSlotCommand::new();
                trace!("EnableSlotCommand pushed");
                self.cr.push(&cmd);
                unsafe { (*self.doorbell_zero).ring(0, 0) };
            }
            Ok(())
        }

        fn address_device(&mut self, port_num: u8, slot_id: u8) -> Result<()> {
            trace!("address_device: port = {}, slot = {}", port_num, slot_id);
            let port = &self.ports[port_num as usize];
            let input_ctx = self.devmgr.add_device(port, slot_id)?;

            self.ports[port_num as usize].set_config_phase(PortConfigPhase::AddressingDevice);
            let cmd = trb::AddressDeviceCommand::new(slot_id, input_ctx as usize);
            self.cr.push(&cmd);
            unsafe { (*self.doorbell_zero).ring(0, 0) };

            Ok(())
        }

        pub fn process_event(&mut self) -> Result<()> {
            let mut er = self.er.take().unwrap();
            if let Some(trb) = er.front() {
                trace!("event found: TRB type = {}", trb.trb_type());
                if let Some(trb) = trb.downcast_ref::<trb::TransferEvent>() {
                    self.on_transfer_event(trb)?;
                } else if let Some(trb) = trb.downcast_ref::<trb::CommandCompletionEvent>() {
                    self.on_command_completion_event(trb)?;
                } else if let Some(trb) = trb.downcast_ref::<trb::PortStatusChangeEvent>() {
                    self.on_port_status_change_event(trb)?;
                }
                er.pop();
                trace!("event popped");
            }
            self.er = Some(er);
            Ok(())
        }

        fn on_transfer_event(&mut self, trb: &trb::TransferEvent) -> Result<()> {
            todo!()
        }
        fn on_command_completion_event(&mut self, trb: &trb::CommandCompletionEvent) -> Result<()> {
            let issuer_type = unsafe { (*trb.command_trb_pointer()).trb_type() };
            let slot_id = trb.slot_id();
            trace!(
                "CommandCompletionEvent: slot_id = {}, issuer trb_type = {}, code = {}",
                slot_id,
                issuer_type,
                trb.completion_code()
            );

            use trb::TrbType;
            if issuer_type == trb::EnableSlotCommand::trb_type() {
                match self.addressing_port {
                    Some(port_num)
                        if self.ports[port_num as usize].config_phase()
                            == PortConfigPhase::EnablingSlot =>
                    {
                        self.address_device(port_num, slot_id)
                    }
                    _ => Err(Error::InvalidPhase),
                }
            } else if issuer_type == trb::AddressDeviceCommand::trb_type() {
                todo!()
            } else if issuer_type == trb::ConfigureEndpointCommand::trb_type() {
                todo!()
            } else {
                Err(Error::InvalidPhase)
            }
        }
        fn on_port_status_change_event(&mut self, trb: &trb::PortStatusChangeEvent) -> Result<()> {
            let port_id = trb.port_id();
            trace!("PortStatusChangeEvent: port_id = {}", port_id);
            match self.ports[port_id as usize].config_phase() {
                PortConfigPhase::NotConnected => self.reset_port(port_id),
                PortConfigPhase::ResettingPort => self.enable_slot(port_id),
                _ => Err(Error::InvalidPhase),
            }
        }
    }

    use trb::Trb;

    struct Ring {
        buf: &'static mut [Trb],
        cycle_bit: bool,
        write_idx: usize,
    }
    impl Ring {
        pub fn with_capacity(buf_size: usize) -> Result<Self> {
            // NOTE: for Transfer Rings, the alignment can be 16-bytes.
            let buf = MALLOC
                .lock()
                .alloc_slice_ext::<Trb>(buf_size, 64, 64 * 1024)
                .ok_or(Error::NoEnoughMemory)?;
            let buf: &mut [MaybeUninit<Trb>] = unsafe { &mut *(buf.as_ptr()) };
            for p in buf.iter_mut() {
                *p = MaybeUninit::zeroed();
            }
            let buf = unsafe { core::mem::transmute::<&mut [MaybeUninit<Trb>], &mut [Trb]>(buf) };
            Ok(Self {
                buf,
                cycle_bit: true,
                write_idx: 0,
            })
        }

        pub fn buffer_ptr(&self) -> *const Trb {
            self.buf.as_ptr()
        }

        fn copy_to_last(&mut self, mut trb: Trb) {
            trb.set_cycle_bit(self.cycle_bit as u8);

            let p = &mut self.buf[self.write_idx] as *mut Trb as *mut u32;

            for i in 0..3 {
                unsafe { p.add(i).write_volatile(trb.data[i]) };
            }
            // NOTE: the last byte of TRB, which contains the cycle bit, must be written atomically.
            unsafe { p.add(3).write_volatile(trb.data[3]) };
        }

        pub fn push<T: trb::TrbType>(&mut self, trb: &T) -> &Trb {
            let trb: &Trb = trb.upcast();
            self.copy_to_last(trb.clone());
            let written_idx = self.write_idx;
            self.write_idx += 1;

            if self.write_idx + 1 == self.buf.len() {
                let mut link = trb::Link::new();
                link.set_toggle_cycle(1);
                let trb = <trb::Link as trb::TrbType>::upcast(&link);
                self.copy_to_last(trb.clone());
                self.write_idx = 0;
                self.cycle_bit = !self.cycle_bit;
            }
            trace!("TRB pushed");
            &self.buf[written_idx]
        }
    }
    // TODO: impl Drop for Ring
    type CommandRing = Ring;
    type TransferRing = Ring;

    struct EventRing {
        buf: *const [Trb],
        erst: *const [EventRingSegmentTableEntry],
        cycle_bit: bool,
        interrupter: *mut InterrupterRegisterSet,
    }
    impl EventRing {
        pub fn with_capacity(buf_size: usize) -> Result<Self> {
            let mut malloc = MALLOC.lock();

            let buf = malloc
                .alloc_slice_ext::<Trb>(buf_size, 64, 64 * 1024)
                .ok_or(Error::NoEnoughMemory)?;
            {
                let buf: &mut [MaybeUninit<Trb>] = unsafe { &mut *buf.as_ptr() };
                for p in buf.iter_mut() {
                    *p = MaybeUninit::zeroed();
                }
            }
            let buf = buf.as_ptr() as *const [Trb];

            let table = malloc
                .alloc_slice_ext::<EventRingSegmentTableEntry>(1, 64, 64 * 1024)
                .ok_or(Error::NoEnoughMemory)?;
            {
                let table: &mut [_] = unsafe { &mut *table.as_ptr() };
                for p in table.iter_mut() {
                    *p = MaybeUninit::zeroed();
                }
                let table = unsafe {
                    core::mem::transmute::<
                        &mut [MaybeUninit<EventRingSegmentTableEntry>],
                        &mut [EventRingSegmentTableEntry],
                    >(table)
                };
                unsafe {
                    table[0].set_pointer((*buf).as_ptr() as usize);
                    table[0].set_ring_segment_size((*buf).len() as u16);
                }
            }
            let table = table.as_ptr() as *const [EventRingSegmentTableEntry];

            Ok(Self {
                buf,
                erst: table,
                cycle_bit: true,
                interrupter: null_mut(),
            })
        }

        pub fn initialize(&mut self, interrupter: *mut InterrupterRegisterSet) {
            self.interrupter = interrupter;

            let (ptr, len) = unsafe { ((*self.erst).as_ptr(), (*self.erst).len()) };

            unsafe {
                (*interrupter).erstsz.modify_with(|erstsz| {
                    erstsz.set_event_ring_segment_table_size(len as u16);
                })
            };

            self.write_dequeue_pointer(unsafe { (*self.buf).as_ptr() });

            unsafe {
                (*interrupter).erstba.modify_with(|erstba| {
                    erstba.set_pointer(ptr as usize);
                })
            };
        }

        pub fn front(&self) -> Option<&Trb> {
            if self.has_front() {
                Some(unsafe { &*self.read_dequeue_pointer() })
            } else {
                None
            }
        }

        pub fn pop(&mut self) {
            let mut new_front = unsafe { self.read_dequeue_pointer().add(1) };

            // TODO: consider multiple segments.
            {
                let begin = unsafe { (*self.erst)[0].pointer() as *const Trb };
                let size = unsafe { (*self.erst)[0].ring_segment_size() };
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
    }
    // TODO: (carefully) impl Drop for EventRing

    #[repr(C, packed(4))]
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

    mod trb {
        // Described in 6.4.6 of the spec.
        #[repr(u8)]
        enum TypeId {
            Link = 6,
            EnableSlotCommand = 9,
            AddressDeviceCommand = 11,
            ConfigureEndpointCommand = 12,

            TransferEvent = 32,
            CommandCompletionEvent = 33,
            PortStatusChangeEvent = 34,
        }

        pub trait TrbType: Sized {
            fn trb_type() -> u8;
            fn upcast(&self) -> &Trb {
                unsafe { core::mem::transmute::<&Self, &Trb>(self) }
            }
        }

        #[repr(C, align(16))]
        #[derive(Clone)]
        pub struct Trb {
            pub data: [u32; 4],
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

            pub fn downcast_ref<T: TrbType>(&self) -> Option<&T> {
                if self.trb_type() == T::trb_type() {
                    Some(unsafe { core::mem::transmute::<&Self, &T>(self) })
                } else {
                    None
                }
            }
            pub fn downcast_mut<T: TrbType>(&mut self) -> Option<&mut T> {
                if self.trb_type() == T::trb_type() {
                    Some(unsafe { core::mem::transmute::<&mut Self, &mut T>(self) })
                } else {
                    None
                }
            }
        }

        #[repr(C, align(16))]
        pub struct Link {
            data: [u32; 4],
        }
        impl Link {
            setter!(data[3]: u32; 0x00000002;  u8, pub set_toggle_cycle);
            setter!(data[3]: u32; 0x0000FC00;  u8, set_trb_type);
            pub fn new() -> Self {
                let mut trb = Self { data: [0; 4] };
                trb.set_trb_type(Self::trb_type() as u8);
                trb
            }
        }
        impl TrbType for Link {
            fn trb_type() -> u8 {
                TypeId::Link as u8
            }
        }

        #[repr(C, align(16))]
        pub struct EnableSlotCommand {
            data: [u32; 4],
        }
        impl EnableSlotCommand {
            setter!(data[3]: u32; 0x0000FC00;  u8, set_trb_type);
            pub fn new() -> Self {
                let mut trb = Self { data: [0; 4] };
                trb.set_trb_type(Self::trb_type() as u8);
                trb
            }
        }
        impl TrbType for EnableSlotCommand {
            fn trb_type() -> u8 {
                TypeId::EnableSlotCommand as u8
            }
        }

        #[repr(C, align(16))]
        pub struct AddressDeviceCommand {
            data: [u32; 4],
        }
        impl AddressDeviceCommand {
            getter!(data[0]: u32; 0xFFFFFFF0; u32, input_ctx_ptr_lo);
            setter!(data[0]: u32; 0xFFFFFFF0; u32, set_input_ctx_ptr_lo);
            getter!(data[1]: u32; 0xFFFFFFFF; u32, input_ctx_ptr_hi);
            setter!(data[1]: u32; 0xFFFFFFFF; u32, set_input_ctx_ptr_hi);

            setter!(data[3]: u32; 0x0000FC00;  u8, set_trb_type);

            getter!(data[3]: u32; 0xFF000000;  u8, slot_id);
            setter!(data[3]: u32; 0xFF000000;  u8, set_slot_id);

            pub fn new(slot_id: u8, input_ctx_ptr: usize) -> Self {
                let mut trb = Self { data: [0; 4] };
                trb.set_trb_type(Self::trb_type() as u8);
                trb.set_input_context_ptr(input_ctx_ptr);
                trb.set_slot_id(slot_id);
                trb
            }

            pub fn input_context_ptr(&self) -> usize {
                (((self.input_ctx_ptr_hi() as u64) << 32) | ((self.input_ctx_ptr_lo() << 4) as u64))
                    as usize
            }
            pub fn set_input_context_ptr(&mut self, ptr: usize) {
                debug_assert!(ptr & 0xF == 0);
                self.set_input_ctx_ptr_lo((((ptr as u64) & 0x00000000FFFFFFFF) >> 4) as u32);
                self.set_input_ctx_ptr_hi((((ptr as u64) & 0xFFFFFFFF00000000) >> 32) as u32);
            }
        }
        impl TrbType for AddressDeviceCommand {
            fn trb_type() -> u8 {
                TypeId::AddressDeviceCommand as u8
            }
        }

        #[repr(C, align(16))]
        pub struct ConfigureEndpointCommand {
            data: [u32; 4],
        }
        impl ConfigureEndpointCommand {
            // TODO
        }
        impl TrbType for ConfigureEndpointCommand {
            fn trb_type() -> u8 {
                TypeId::ConfigureEndpointCommand as u8
            }
        }

        #[repr(C, align(16))]
        pub struct TransferEvent {
            data: [u32; 4],
        }
        impl TrbType for TransferEvent {
            fn trb_type() -> u8 {
                TypeId::TransferEvent as u8
            }
        }

        #[repr(C, align(16))]
        pub struct CommandCompletionEvent {
            data: [u32; 4],
        }
        impl CommandCompletionEvent {
            getter!(data[0]: u32; 0xFFFFFFF0; u32, command_trb_pointer_lo);
            getter!(data[1]: u32; 0xFFFFFFFF; u32, command_trb_pointer_hi);
            getter!(data[2]: u32; 0xFF000000;  u8, pub completion_code);
            getter!(data[3]: u32; 0xFF000000;  u8, pub slot_id);

            pub fn command_trb_pointer(&self) -> *const Trb {
                let lo = self.command_trb_pointer_lo() << 4;
                let hi = self.command_trb_pointer_hi();
                (((hi as usize) << 32) | (lo as usize)) as *const Trb
            }
        }
        impl TrbType for CommandCompletionEvent {
            fn trb_type() -> u8 {
                TypeId::CommandCompletionEvent as u8
            }
        }

        #[repr(C, align(16))]
        pub struct PortStatusChangeEvent {
            data: [u32; 4],
        }
        impl TrbType for PortStatusChangeEvent {
            fn trb_type() -> u8 {
                TypeId::PortStatusChangeEvent as u8
            }
        }
        impl PortStatusChangeEvent {
            getter!(data[0]: u32; 0xFF000000; u8, pub port_id);
        }
    }

    #[repr(C, align(32))]
    struct SlotContext {
        data: [u32; 8],
    }
    impl SlotContext {
        getter!(data[0]: u32; 0x000FFFFF; u32, pub route_string);
        setter!(data[0]: u32; 0x000FFFFF; u32, pub set_route_string);

        getter!(data[0]: u32; 0x00F00000;  u8, pub speed);
        setter!(data[0]: u32; 0x00F00000;  u8, pub set_speed);

        getter!(data[0]: u32; 0xF8000000;  u8, pub context_entries);
        setter!(data[0]: u32; 0xF8000000;  u8, pub set_context_entries);

        getter!(data[1]: u32; 0x00FF0000;  u8, pub root_hub_port_number);
        setter!(data[1]: u32; 0x00FF0000;  u8, pub set_root_hub_port_number);
    }

    #[repr(u8)]
    enum EndpointType {
        ControllBidirectional = 4,
    }

    #[repr(C, align(32))]
    struct EndpointContext {
        data: [u32; 8],
    }
    impl EndpointContext {
        getter!(data[0]: u32; 0x00000300;  u8, pub mult);
        setter!(data[0]: u32; 0x00000300;  u8, pub set_mult);

        getter!(data[0]: u32; 0x00007C00;  u8, pub max_primary_streams);
        setter!(data[0]: u32; 0x00007C00;  u8, pub set_max_primary_streams);

        getter!(data[0]: u32; 0x00FF0000;  u8, pub interval);
        setter!(data[0]: u32; 0x00FF0000;  u8, pub set_interval);

        getter!(data[1]: u32; 0x00000006;  u8, pub error_count);
        setter!(data[1]: u32; 0x00000006;  u8, pub set_error_count);

        getter!(data[1]: u32; 0x00000038;  u8, pub endpoint_type);
        setter!(data[1]: u32; 0x00000038;  u8, pub set_endpoint_type);

        getter!(data[1]: u32; 0x0000FF00;  u8, pub max_burst_size);
        setter!(data[1]: u32; 0x0000FF00;  u8, pub set_max_burst_size);

        getter!(data[1]: u32; 0xFFFF0000; u16, pub max_packet_size);
        setter!(data[1]: u32; 0xFFFF0000; u16, pub set_max_packet_size);

        getter!(data[2]: u32; 0x00000001;  u8, pub dequeue_cycle_state);
        setter!(data[2]: u32; 0x00000001;  u8, pub set_dequeue_cycle_state);

        getter!(data[2]: u32; 0xFFFFFFF0; u32, dequeue_pointer_lo);
        setter!(data[2]: u32; 0xFFFFFFF0; u32, set_dequeue_pointer_lo);
        getter!(data[3]: u32; 0xFFFFFFFF; u32, dequeue_pointer_hi);
        setter!(data[3]: u32; 0xFFFFFFFF; u32, set_dequeue_pointer_hi);

        pub fn set_transfer_ring_buffer(&mut self, ptr: usize) {
            self.set_dequeue_pointer_lo((((ptr as u64) & 0x00000000FFFFFFFF) >> 04) as u32);
            self.set_dequeue_pointer_hi((((ptr as u64) & 0xFFFFFFFF00000000) >> 32) as u32);
        }
    }

    #[repr(transparent)]
    #[derive(Clone, Copy)]
    struct DeviceContextIndex(usize);
    impl DeviceContextIndex {
        pub fn new_out(ep_num: u8) -> Self {
            Self((ep_num as usize) * 2 + (ep_num == 0) as usize)
        }
        pub fn new_in(ep_num: u8) -> Self {
            Self((ep_num as usize) * 2 + 1)
        }
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

    #[repr(C, align(32))]
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
        input_control_ctx: InputControlContext,
        slot_ctx: SlotContext,
        ep_ctxs: [EndpointContext; 31],
    }
    impl InputContext {
        pub unsafe fn initialize_ptr(ptr: *mut Self) {
            let ptr = ptr as *mut u8;
            ptr.write_bytes(0, size_of::<Self>());
        }
        pub fn enable_slot_context(&mut self) -> &mut SlotContext {
            assert_eq!(size_of::<InputControlContext>(), 32);
            assert_eq!(size_of::<SlotContext>(), 32);
            self.input_control_ctx.add_context_flags |= 1;
            &mut self.slot_ctx
        }
        pub fn enable_endpoint(&mut self, dci: DeviceContextIndex) -> &mut EndpointContext {
            self.input_control_ctx.add_context_flags |= 1 << dci.0;
            &mut self.ep_ctxs[dci.0 - 1]
        }
    }

    enum DeviceState {
        Invalid,
        Blank,
        SlotAssigning,
        SlotAssigned,
    }

    struct Device {
        ctx: *const DeviceContext,
        input_ctx: InputContext,
        state: DeviceState,
        doorbell: *mut DoorbellRegister,
        transfer_rings: [Option<TransferRing>; 31],
    }
    impl Device {
        pub unsafe fn initialize_ptr(
            ptr: *mut Self,
            ctx: *const DeviceContext,
            doorbell: *mut DoorbellRegister,
            port: &Port,
        ) -> Result<*const InputContext> {
            {
                let ctx_ptr = addr_of_mut!((*ptr).ctx);
                ctx_ptr.write(ctx);

                let input_ctx_ptr = addr_of_mut!((*ptr).input_ctx);
                InputContext::initialize_ptr(input_ctx_ptr);

                let state_ptr = addr_of_mut!((*ptr).state);
                state_ptr.write(DeviceState::Blank);

                let doorbell_ptr = addr_of_mut!((*ptr).doorbell);
                doorbell_ptr.write(doorbell);

                let transfer_rings_ptr =
                    addr_of_mut!((*ptr).transfer_rings) as *mut Option<TransferRing>;
                for i in 0..31 {
                    unsafe { transfer_rings_ptr.add(i).write(None) };
                }
            }
            let device = unsafe { &mut *ptr };

            let slot_ctx = device.input_ctx.enable_slot_context();
            slot_ctx.set_route_string(0);
            slot_ctx.set_root_hub_port_number(port.number());
            slot_ctx.set_context_entries(1);
            slot_ctx.set_speed(port.speed() as u8);

            let ep0_dci = DeviceContextIndex::new_out(0);
            let tr_buf = device.alloc_transfer_ring(ep0_dci, 32)?.buffer_ptr();
            let max_packet_size = port.speed().determine_max_packet_size_for_control_pipe();

            let ep0_ctx = device.input_ctx.enable_endpoint(ep0_dci);
            ep0_ctx.set_endpoint_type(EndpointType::ControllBidirectional as u8);
            ep0_ctx.set_max_packet_size(max_packet_size);
            ep0_ctx.set_max_burst_size(0);
            ep0_ctx.set_transfer_ring_buffer(tr_buf as usize);
            ep0_ctx.set_dequeue_cycle_state(1);
            ep0_ctx.set_interval(0);
            ep0_ctx.set_max_primary_streams(0);
            ep0_ctx.set_mult(0);
            ep0_ctx.set_error_count(3);

            Ok(&device.input_ctx)
        }

        fn alloc_transfer_ring(
            &mut self,
            dci: DeviceContextIndex,
            capacity: usize,
        ) -> Result<&TransferRing> {
            let i = dci.0 - 1;
            self.transfer_rings[i] = Some(TransferRing::with_capacity(capacity)?);
            Ok(self.transfer_rings[i].as_ref().unwrap())
        }
    }

    struct DeviceManager {
        devices: &'static mut [*mut Device],
        device_context_pointers: *mut [*const DeviceContext],
        doorbells: *mut DoorbellRegister, // 1-255
    }
    impl DeviceManager {
        pub fn new(max_slots: usize, doorbells: *mut DoorbellRegister) -> Result<Self> {
            let mut malloc = MALLOC.lock();

            let devices = malloc
                .alloc_slice::<*mut Device>(max_slots + 1)
                .ok_or(Error::NoEnoughMemory)?;
            let devices: &mut [MaybeUninit<*mut Device>] = unsafe { &mut *devices.as_ptr() };
            for p in devices.iter_mut() {
                *p = MaybeUninit::new(null_mut());
            }
            let devices = unsafe {
                core::mem::transmute::<&mut [MaybeUninit<*mut Device>], &mut [*mut Device]>(devices)
            };

            // NOTE: DCBAA: alignment = 64-bytes, boundary = PAGESIZE
            let bound = malloc.default_boundary;
            let dcbaap = malloc
                .alloc_slice_ext::<*const DeviceContext>(max_slots + 1, 64, bound)
                .ok_or(Error::NoEnoughMemory)?;
            {
                let dcbaap: &mut [MaybeUninit<*const DeviceContext>] =
                    unsafe { &mut *dcbaap.as_ptr() };
                for p in dcbaap.iter_mut() {
                    *p = MaybeUninit::new(null());
                }
            }
            let dcbaap = dcbaap.as_ptr() as *mut [*const DeviceContext];

            debug!(
                "DeviceManager has been initialized for up to {} devices",
                max_slots
            );

            Ok(Self {
                devices,
                device_context_pointers: dcbaap,
                doorbells,
            })
        }

        pub fn add_device(&mut self, port: &Port, slot_id: u8) -> Result<*const InputContext> {
            let slot_id = slot_id as usize;
            if !(1 <= slot_id && slot_id < self.devices.len()) {
                return Err(Error::InvalidSlotId);
            }
            if !self.devices[slot_id].is_null() {
                return Err(Error::DeviceAlreadyAllocated);
            }

            let device_ctx = MALLOC
                .lock()
                .alloc_obj::<DeviceContext>()
                .ok_or(Error::NoEnoughMemory)?;
            let device_ctx = device_ctx.as_ptr();
            unsafe { DeviceContext::initialize_ptr((*device_ctx).as_mut_ptr()) };
            let device_ctx = device_ctx as *const DeviceContext;

            let device = MALLOC
                .lock()
                .alloc_obj::<Device>()
                .ok_or(Error::NoEnoughMemory)?;
            let device = unsafe { (*device.as_ptr()).as_mut_ptr() };

            let input_ctx = unsafe {
                Device::initialize_ptr(device, device_ctx, self.doorbells.add(slot_id - 1), port)?
            };

            self.devices[slot_id] = device;
            unsafe {
                (*self.device_context_pointers)
                    .as_mut_ptr()
                    .add(slot_id)
                    .write(device_ctx)
            };
            trace!("add_device: slot_id = {}", slot_id);

            Ok(input_ctx)
        }

        pub fn dcbaap(&self) -> *const *const DeviceContext {
            let ptr = unsafe { (*self.device_context_pointers).as_ptr() };
            assert!((ptr as usize) % 64 == 0);
            ptr
        }
    }
    // TODO: impl Drop for DeviceManager

    #[repr(u8)]
    pub enum PortSpeed {
        Full = 1,
        Low = 2,
        High = 3,
        Super = 4,
        SuperPlus = 5,
    }
    impl PortSpeed {
        pub fn determine_max_packet_size_for_control_pipe(&self) -> u16 {
            match self {
                Self::Super => 512,
                Self::High => 64,
                _ => 8,
            }
        }
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

        pub fn clear_connect_status_change(&mut self) {
            unsafe {
                (*self.regs).portsc.modify_with(|portsc| {
                    portsc.data &= 0x0E01C3E0;
                    portsc.set_connect_status_change(1);
                })
            };
        }

        pub fn clear_port_reset_change(&mut self) {
            unsafe {
                (*self.regs).portsc.modify_with(|portsc| {
                    portsc.data &= 0x0E01C3E0;
                    portsc.set_port_reset_change(1);
                })
            };
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

    use crate::sync::spin::SpinMutex;
    use crate::utils::StaticMallocator;
    const BUF_SIZE: usize = 4096 * 32;
    static MALLOC: SpinMutex<StaticMallocator<BUF_SIZE>> =
        SpinMutex::new("usb/malloc", StaticMallocator::new());
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
