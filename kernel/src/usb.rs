#![allow(clippy::transmute_ptr_to_ptr)]
#![allow(clippy::identity_op)]

#[derive(Debug)]
pub enum Error {
    NoEnoughMemory,
    InvalidPhase,
    InvalidSlotId,
    InvalidEndpointNumber,
    DeviceAlreadyAllocated,
    TransferRingNotSet,
    TransferFailed,
    NoCorrespondingSetupStage,
    NoWaiter,
    TooManyWaiters,
    UnsupportedInterface,
}
pub type Result<T> = core::result::Result<T, Error>;

pub mod xhci {
    use super::{Error, Result};
    use core::mem::{size_of, MaybeUninit};
    use core::ptr::{addr_of_mut, null, null_mut, NonNull};

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

    #[repr(C)]
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
        er: EventRing,
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

            if unsafe { (*op_regs).usbsts.read().host_controller_halted() == 0 } {
                unsafe {
                    (*op_regs)
                        .usbcmd
                        .modify_with(|usbcmd| usbcmd.set_run_stop(0))
                };
            }

            // Host controller must be halted
            while unsafe { (*op_regs).usbsts.read().host_controller_halted() == 0 } {}
            debug!("host controller halted");

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
            // Set "Max Slots Enabled" field in CONFIG
            unsafe {
                (*op_regs).config.modify_with(|config| {
                    config.set_max_device_slots_enabled(slots);
                })
            };
            let devmgr = DeviceManager::new(slots as usize, unsafe { doorbell_zero.add(1) })?;

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
                ports[0] = MaybeUninit::zeroed();
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
                er,
                ports,
                max_ports,
                addressing_port: None,
                doorbell_zero,
            })
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

        pub fn configure_ports(&mut self) {
            trace!("configure_ports");
            let mut first = None;
            for port_num in 1..=self.max_ports {
                if !self.ports[port_num as usize].is_connected() {
                    continue;
                }
                if first.is_none() {
                    first = Some(port_num);
                }
                debug!("Port {}: connected", port_num);
                if self.ports[port_num as usize].config_phase() == PortConfigPhase::NotConnected {
                    if let Err(e) = self.reset_port(port_num) {
                        error!("Failed to configure the port {}: {:?}", port_num, e);
                    }
                }
            }
            for port_num in 1..=self.max_ports {
                if !self.ports[port_num as usize].is_connected() {
                    continue;
                }
                if first == Some(port_num) {
                    assert_eq!(
                        self.ports[port_num as usize].config_phase(),
                        PortConfigPhase::ResettingPort
                    );
                } else {
                    assert_eq!(
                        self.ports[port_num as usize].config_phase(),
                        PortConfigPhase::WaitingAddressed
                    );
                }
                debug!(
                    "Port {}: {:?}",
                    port_num,
                    self.ports[port_num as usize].config_phase()
                );
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
                        warn!(
                            "port.config_phase() = {:?} (should be {:?} or {:?})",
                            port.config_phase(),
                            PortConfigPhase::NotConnected,
                            PortConfigPhase::WaitingAddressed
                        );
                        return Err(Error::InvalidPhase);
                    }
                    port.set_config_phase(PortConfigPhase::ResettingPort);
                    port.reset();
                }
            }
            Ok(())
        }

        fn ring_doorbell(&mut self) {
            unsafe { (*self.doorbell_zero).ring(0, 0) };
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
                let cmd = trb::EnableSlotCommand::default();
                trace!("EnableSlotCommand pushed");
                self.cr.push(&cmd);
                self.ring_doorbell();
            }
            Ok(())
        }

        fn address_device(&mut self, port_num: u8, slot_id: u8) -> Result<()> {
            trace!("address_device: port = {}, slot = {}", port_num, slot_id);
            let port = &self.ports[port_num as usize];
            let input_ctx = self.devmgr.add_device(port, slot_id)?;

            self.ports[port_num as usize].set_config_phase(PortConfigPhase::AddressingDevice);
            let mut cmd = trb::AddressDeviceCommand::default();
            cmd.set_input_context_ptr(input_ctx as usize);
            cmd.set_slot_id(slot_id);
            self.cr.push(&cmd);
            self.ring_doorbell();

            Ok(())
        }

        fn initialize_device(&mut self, port_num: u8, slot_id: u8) -> Result<()> {
            trace!("initialize_device: port = {}, slot = {}", port_num, slot_id);

            let dev = self
                .devmgr
                .find_by_slot_mut(slot_id)
                .ok_or(Error::InvalidSlotId)?;
            let port_num = dev.port_num();
            self.ports[port_num as usize].set_config_phase(PortConfigPhase::InitializingDevice);

            dev.start_initialize()
        }

        pub fn process_event(&mut self) -> Result<()> {
            if let Some(trb) = self.er.front() {
                trace!("event found: TRB type = {}", trb.trb_type());

                use trb::Trb;
                match trb.trb_type() {
                    trb::TransferEvent::TYPE => self.on_transfer_event()?,
                    trb::CommandCompletionEvent::TYPE => self.on_command_completion_event()?,
                    trb::PortStatusChangeEvent::TYPE => self.on_port_status_change_event()?,
                    _ => {}
                }

                self.er.pop();
                trace!("event popped");
            }
            Ok(())
        }

        fn on_transfer_event(&mut self) -> Result<()> {
            let trb = self
                .er
                .front()
                .unwrap()
                .downcast_ref::<trb::TransferEvent>()
                .expect("TRB is not a TransferEvent");

            let slot_id = trb.slot_id();
            trace!("TransferEvent: slot_id = {}", slot_id);

            let dev = self
                .devmgr
                .find_by_slot_mut(slot_id)
                .ok_or(Error::InvalidSlotId)?;

            dev.on_transfer_event_received(trb)?;

            if dev.is_initialized()
                && self.ports[dev.port_num() as usize].config_phase()
                    == PortConfigPhase::InitializingDevice
            {
                unimplemented!();
            }

            Ok(())
        }
        fn on_command_completion_event(&mut self) -> Result<()> {
            let trb = self
                .er
                .front()
                .unwrap()
                .downcast_ref::<trb::CommandCompletionEvent>()
                .expect("TRB is not a CommandCompletionEvent");

            let issuer_type = unsafe { (*trb.command_trb_pointer()).trb_type() };
            let slot_id = trb.slot_id();
            trace!(
                "CommandCompletionEvent: slot_id = {}, issuer trb_type = {}, code = {}",
                slot_id,
                issuer_type,
                trb.completion_code()
            );

            use trb::Trb;
            match issuer_type {
                trb::EnableSlotCommand::TYPE => match self.addressing_port {
                    Some(port_num)
                        if self.ports[port_num as usize].config_phase()
                            == PortConfigPhase::EnablingSlot =>
                    {
                        self.address_device(port_num, slot_id)
                    }
                    _ => {
                        warn!("addression_port is None");
                        Err(Error::InvalidPhase)
                    }
                },
                trb::AddressDeviceCommand::TYPE => {
                    let dev = self
                        .devmgr
                        .find_by_slot(slot_id)
                        .ok_or(Error::InvalidSlotId)?;
                    let port_num = dev.port_num();
                    if self.addressing_port.unwrap_or(0) != port_num
                        || self.ports[port_num as usize].config_phase()
                            != PortConfigPhase::AddressingDevice
                    {
                        if self.addressing_port != Some(port_num) {
                            warn!(
                                "addression_port = {:?}, but the event is on port = {}",
                                self.addressing_port, port_num
                            );
                        } else {
                            warn!(
                                "ports[{}].config_phase() = {:?}",
                                port_num,
                                self.ports[port_num as usize].config_phase(),
                            );
                        }
                        Err(Error::InvalidPhase)
                    } else {
                        self.addressing_port = None;
                        for i in 1..=self.max_ports {
                            if self.ports[i as usize].config_phase()
                                == PortConfigPhase::WaitingAddressed
                            {
                                self.reset_port(i)?;
                                break;
                            }
                        }
                        self.initialize_device(port_num, slot_id)
                    }
                }
                trb::ConfigureEndpointCommand::TYPE => {
                    unimplemented!();
                }
                _ => {
                    warn!("unexpected Event");
                    Err(Error::InvalidPhase)
                }
            }
        }
        fn on_port_status_change_event(&mut self) -> Result<()> {
            let trb = self
                .er
                .front()
                .unwrap()
                .downcast_ref::<trb::PortStatusChangeEvent>()
                .expect("TRB is not a PortStatusChangeEvent");

            let port_id = trb.port_id();
            let port = &self.ports[port_id as usize];
            trace!(
                "PortStatusChangeEvent: port_id = {}, (CSC = {}, CCS = {}, PRC = {})",
                port_id,
                port.is_connect_status_changed(),
                port.is_connected(),
                port.is_port_reset_changed(),
            );
            match port.config_phase() {
                PortConfigPhase::NotConnected => {
                    if port.is_connect_status_changed() {
                        self.reset_port(port_id)
                    } else {
                        trace!("skipping: port_id = {}", port_id);
                        Ok(())
                    }
                }
                PortConfigPhase::ResettingPort => {
                    if port.is_port_reset_changed() {
                        self.enable_slot(port_id)
                    } else {
                        trace!("skipping: port_id = {}", port_id);
                        Ok(())
                    }
                }
                PortConfigPhase::WaitingAddressed => {
                    trace!("waiting addressed: port_id = {}", port_id);
                    Ok(())
                }
                phase => {
                    warn!(
                        "config_phase = {:?} (should be {:?} or {:?})",
                        phase,
                        PortConfigPhase::NotConnected,
                        PortConfigPhase::ResettingPort,
                    );
                    Err(Error::InvalidPhase)
                }
            }
        }
    }

    use trb::GenericTrb;

    struct Ring {
        buf: &'static mut [GenericTrb],
        cycle_bit: bool,
        write_idx: usize,
    }
    impl Ring {
        pub fn with_capacity(buf_size: usize) -> Result<Self> {
            // NOTE: for Transfer Rings, the alignment can be 16-bytes.
            let buf = MALLOC
                .lock()
                .alloc_slice_ext::<GenericTrb>(buf_size, 64, 64 * 1024)
                .ok_or(Error::NoEnoughMemory)?;
            let buf: &mut [MaybeUninit<GenericTrb>] = unsafe { &mut *(buf.as_ptr()) };
            for p in buf.iter_mut() {
                *p = MaybeUninit::zeroed();
            }
            let buf = unsafe {
                core::mem::transmute::<&mut [MaybeUninit<GenericTrb>], &mut [GenericTrb]>(buf)
            };
            Ok(Self {
                buf,
                cycle_bit: true,
                write_idx: 0,
            })
        }

        pub fn buffer_ptr(&self) -> *const GenericTrb {
            self.buf.as_ptr()
        }

        fn copy_to_last(&mut self, mut trb: GenericTrb) {
            trb.set_cycle_bit(self.cycle_bit as u8);

            let p = &mut self.buf[self.write_idx] as *mut GenericTrb as *mut u32;

            for i in 0..3 {
                unsafe { p.add(i).write_volatile(trb.data[i]) };
            }
            // NOTE: the last byte of TRB, which contains the cycle bit, must be written atomically.
            unsafe { p.add(3).write_volatile(trb.data[3]) };
        }

        pub fn push<T: trb::Trb>(&mut self, trb: &T) -> &GenericTrb {
            let trb: &GenericTrb = trb.upcast();
            self.copy_to_last(trb.clone());
            let written_idx = self.write_idx;
            self.write_idx += 1;

            if self.write_idx + 1 == self.buf.len() {
                let mut link = trb::Link::default();
                link.set_toggle_cycle(1);
                let trb = <trb::Link as trb::Trb>::upcast(&link);
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
        buf: *const [GenericTrb],
        erst: *const [EventRingSegmentTableEntry],
        cycle_bit: bool,
        interrupter: *mut InterrupterRegisterSet,
    }
    impl EventRing {
        pub fn with_capacity(buf_size: usize) -> Result<Self> {
            let mut malloc = MALLOC.lock();

            let buf = malloc
                .alloc_slice_ext::<GenericTrb>(buf_size, 64, 64 * 1024)
                .ok_or(Error::NoEnoughMemory)?;
            {
                let buf: &mut [MaybeUninit<GenericTrb>] = unsafe { &mut *buf.as_ptr() };
                for p in buf.iter_mut() {
                    *p = MaybeUninit::zeroed();
                }
            }
            let buf = buf.as_ptr() as *const [GenericTrb];

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

        pub fn front(&self) -> Option<&GenericTrb> {
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
                let begin = unsafe { (*self.erst)[0].pointer() as *const GenericTrb };
                let size = unsafe { (*self.erst)[0].ring_segment_size() };
                let end = unsafe { begin.add(size as usize) };

                if new_front == end {
                    new_front = begin;
                    self.cycle_bit = !self.cycle_bit;
                }
            }

            self.write_dequeue_pointer(new_front);
        }

        fn write_dequeue_pointer(&mut self, ptr: *const GenericTrb) {
            unsafe {
                (*self.interrupter).erdp.modify_with(|erdp| {
                    erdp.set_pointer(ptr as usize);
                })
            };
        }

        fn read_dequeue_pointer(&self) -> *const GenericTrb {
            unsafe { (*self.interrupter).erdp.read().pointer() as *const GenericTrb }
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
            Normal = 1,

            SetupStage = 2,
            DataStage = 3,
            StatusStage = 4,
            Link = 6,

            EnableSlotCommand = 9,
            AddressDeviceCommand = 11,
            ConfigureEndpointCommand = 12,

            TransferEvent = 32,
            CommandCompletionEvent = 33,
            PortStatusChangeEvent = 34,
        }

        pub trait Trb: Sized {
            const TYPE: u8;

            fn upcast(&self) -> &GenericTrb {
                unsafe { core::mem::transmute::<&Self, &GenericTrb>(self) }
            }
        }

        #[repr(C, align(16))]
        #[derive(Clone)]
        pub struct GenericTrb {
            pub data: [u32; 4],
        }
        impl GenericTrb {
            getter!(data[2]: u32; 0xFFFFFFFF; u32, status);
            setter!(data[2]: u32; 0xFFFFFFFF; u32, set_status);

            getter!(data[3]: u32; 0x00000001;  u8, pub cycle_bit);
            setter!(data[3]: u32; 0x00000001;  u8, pub set_cycle_bit);

            getter!(data[3]: u32; 0x00000002;  u8, evaluate_next_trb);
            setter!(data[3]: u32; 0x00000002;  u8, set_evaluate_next_trb);

            getter!(data[3]: u32; 0x0000FC00;  u8, pub trb_type);
            setter!(data[3]: u32; 0x0000FC00;  u8, set_trb_type);

            getter!(data[3]: u32; 0xFFFF0000; u16, control);
            setter!(data[3]: u32; 0xFFFF0000; u16, set_control);

            pub fn downcast_ref<T: Trb>(&self) -> Option<&T> {
                if self.trb_type() == T::TYPE {
                    Some(unsafe { core::mem::transmute::<&Self, &T>(self) })
                } else {
                    None
                }
            }
            pub fn downcast_mut<T: Trb>(&mut self) -> Option<&mut T> {
                if self.trb_type() == T::TYPE {
                    Some(unsafe { core::mem::transmute::<&mut Self, &mut T>(self) })
                } else {
                    None
                }
            }
        }
        impl core::fmt::Debug for GenericTrb {
            fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                let data = &self.data;
                write!(
                    f,
                    "TRB: {:08x} {:08x} {:08x} {:08x}",
                    data[0], data[1], data[2], data[3]
                )
            }
        }

        #[repr(C, align(16))]
        pub struct Normal {
            data: [u32; 4],
        }
        impl Trb for Normal {
            const TYPE: u8 = TypeId::Normal as u8;
        }

        #[repr(C, align(16))]
        pub struct SetupStage {
            data: [u32; 4],
        }
        impl SetupStage {
            getter!(data[0]: u32; 0x000000FF;  u8, pub request_type);
            setter!(data[0]: u32; 0x000000FF;  u8, pub set_request_type);

            getter!(data[0]: u32; 0x0000FF00;  u8, pub request);
            setter!(data[0]: u32; 0x0000FF00;  u8, pub set_request);

            getter!(data[0]: u32; 0xFFFF0000; u16, pub value);
            setter!(data[0]: u32; 0xFFFF0000; u16, pub set_value);

            getter!(data[1]: u32; 0x0000FFFF; u16, pub index);
            setter!(data[1]: u32; 0x0000FFFF; u16, pub set_index);

            getter!(data[1]: u32; 0xFFFF0000; u16, pub length);
            setter!(data[1]: u32; 0xFFFF0000; u16, pub set_length);

            getter!(data[2]: u32; 0x0001FFFF; u32, trb_transfer_length);
            setter!(data[2]: u32; 0x0001FFFF; u32, set_trb_transfer_length);

            getter!(data[3]: u32; 0x00000040;  u8, pub immediate_data);
            setter!(data[3]: u32; 0x00000040;  u8, pub set_immediate_data);

            setter!(data[3]: u32; 0x0000FC00;  u8, set_trb_type);

            getter!(data[3]: u32; 0x00030000;  u8, pub transfer_type);
            setter!(data[3]: u32; 0x00030000;  u8, pub set_transfer_type);

            fn new(setup_data: super::SetupData, transfer_type: u8) -> Self {
                let mut setup = Self::default();
                setup.set_request_type(setup_data.request_type);
                setup.set_request(setup_data.request);
                setup.set_value(setup_data.value);
                setup.set_index(setup_data.index);
                setup.set_length(setup_data.length);
                setup.set_transfer_type(transfer_type);
                setup
            }
            pub fn new_no_data_stage(setup_data: super::SetupData) -> Self {
                Self::new(setup_data, 0)
            }
            pub fn new_out_data_stage(setup_data: super::SetupData) -> Self {
                Self::new(setup_data, 2)
            }
            pub fn new_in_data_stage(setup_data: super::SetupData) -> Self {
                Self::new(setup_data, 3)
            }
        }
        impl Default for SetupStage {
            fn default() -> Self {
                let mut trb = Self { data: [0; 4] };
                trb.set_trb_type(Self::TYPE);
                trb.set_immediate_data(1);
                trb.set_trb_transfer_length(8);
                trb
            }
        }
        impl Trb for SetupStage {
            const TYPE: u8 = TypeId::SetupStage as u8;
        }

        #[repr(C, align(16))]
        pub struct DataStage {
            data: [u32; 4],
        }
        impl DataStage {
            getter!(data[0]: u32; 0xFFFFFFFF; u32, data_buffer_lo);
            setter!(data[0]: u32; 0xFFFFFFFF; u32, set_data_buffer_lo);
            getter!(data[1]: u32; 0xFFFFFFFF; u32, data_buffer_hi);
            setter!(data[1]: u32; 0xFFFFFFFF; u32, set_data_buffer_hi);

            getter!(data[2]: u32; 0x0001FFFF; u32, pub trb_transfer_length);
            setter!(data[2]: u32; 0x0001FFFF; u32, pub set_trb_transfer_length);

            getter!(data[2]: u32; 0x003E0000; u32, pub td_size);
            setter!(data[2]: u32; 0x003E0000; u32, pub set_td_size);

            getter!(data[3]: u32; 0x00000020;  u8, pub interrupt_on_completion);
            setter!(data[3]: u32; 0x00000020;  u8, pub set_interrupt_on_completion);

            setter!(data[3]: u32; 0x0000FC00;  u8, set_trb_type);

            getter!(data[3]: u32; 0x00010000;  u8, pub direction);
            setter!(data[3]: u32; 0x00010000;  u8, pub set_direction);

            pub fn data_buffer(&self) -> *const u8 {
                (((self.data_buffer_hi() as usize) << 32) | (self.data_buffer_lo() as usize))
                    as *const u8
            }
            pub fn set_data_buffer(&mut self, ptr: *const u8) {
                let ptr = ptr as usize;
                self.set_data_buffer_hi(((ptr & 0xFFFFFFFF00000000) >> 32) as u32);
                self.set_data_buffer_lo(((ptr & 0x00000000FFFFFFFF) >> 00) as u32);
            }

            fn new(buf: &[u8], dir_in: bool) -> Self {
                let mut trb = Self::default();
                trb.set_data_buffer(buf.as_ptr());
                trb.set_trb_transfer_length(buf.len() as u32);
                trb.set_td_size(0);
                trb.set_direction(dir_in as u8);
                trb
            }
            pub fn new_out(buf: &[u8]) -> Self {
                Self::new(buf, false)
            }
            pub fn new_in(buf: &[u8]) -> Self {
                Self::new(buf, true)
            }
        }
        impl Default for DataStage {
            fn default() -> Self {
                let mut trb = Self { data: [0; 4] };
                trb.set_trb_type(Self::TYPE);
                trb
            }
        }
        impl Trb for DataStage {
            const TYPE: u8 = TypeId::DataStage as u8;
        }

        #[repr(C, align(16))]
        pub struct StatusStage {
            data: [u32; 4],
        }
        impl StatusStage {
            getter!(data[3]: u32; 0x00000020; u8, pub interrupt_on_completion);
            setter!(data[3]: u32; 0x00000020; u8, pub set_interrupt_on_completion);

            setter!(data[3]: u32; 0x0000FC00; u8, set_trb_type);

            getter!(data[3]: u32; 0x00010000; u8, pub direction);
            setter!(data[3]: u32; 0x00010000; u8, pub set_direction);
        }
        impl Default for StatusStage {
            fn default() -> Self {
                let mut trb = Self { data: [0; 4] };
                trb.set_trb_type(Self::TYPE);
                trb
            }
        }
        impl Trb for StatusStage {
            const TYPE: u8 = TypeId::StatusStage as u8;
        }

        #[repr(C, align(16))]
        pub struct Link {
            data: [u32; 4],
        }
        impl Link {
            setter!(data[3]: u32; 0x00000002; u8, pub set_toggle_cycle);
            setter!(data[3]: u32; 0x0000FC00; u8, set_trb_type);
        }
        impl Default for Link {
            fn default() -> Self {
                let mut trb = Self { data: [0; 4] };
                trb.set_trb_type(Self::TYPE);
                trb
            }
        }
        impl Trb for Link {
            const TYPE: u8 = TypeId::Link as u8;
        }

        #[repr(C, align(16))]
        pub struct EnableSlotCommand {
            data: [u32; 4],
        }
        impl EnableSlotCommand {
            setter!(data[3]: u32; 0x0000FC00; u8, set_trb_type);
        }
        impl Default for EnableSlotCommand {
            fn default() -> Self {
                let mut trb = Self { data: [0; 4] };
                trb.set_trb_type(Self::TYPE);
                trb
            }
        }
        impl Trb for EnableSlotCommand {
            const TYPE: u8 = TypeId::EnableSlotCommand as u8;
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

            getter!(data[3]: u32; 0xFF000000;  u8, pub slot_id);
            setter!(data[3]: u32; 0xFF000000;  u8, pub set_slot_id);

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
        impl Default for AddressDeviceCommand {
            fn default() -> Self {
                let mut trb = Self { data: [0; 4] };
                trb.set_trb_type(Self::TYPE);
                trb
            }
        }
        impl Trb for AddressDeviceCommand {
            const TYPE: u8 = TypeId::AddressDeviceCommand as u8;
        }

        #[repr(C, align(16))]
        pub struct ConfigureEndpointCommand {
            data: [u32; 4],
        }
        impl ConfigureEndpointCommand {
            // TODO
        }
        impl Trb for ConfigureEndpointCommand {
            const TYPE: u8 = TypeId::ConfigureEndpointCommand as u8;
        }

        #[repr(C, align(16))]
        pub struct TransferEvent {
            data: [u32; 4],
        }
        impl TransferEvent {
            getter!(data[0]: u32; 0xFFFFFFFF; u32, trb_pointer_lo);
            getter!(data[1]: u32; 0xFFFFFFFF; u32, trb_pointer_hi);

            getter!(data[2]: u32; 0x00FFFFFF; u32, pub trb_transfer_length);
            getter!(data[2]: u32; 0xFF000000;  u8, pub completion_code);

            getter!(data[3]: u32; 0x001F0000;  u8, pub endpoint_id);
            getter!(data[3]: u32; 0xFF000000;  u8, pub slot_id);

            pub fn trb_pointer(&self) -> *const GenericTrb {
                (((self.trb_pointer_hi() as usize) << 32) | (self.trb_pointer_lo() as usize))
                    as *const GenericTrb
            }
        }
        impl Trb for TransferEvent {
            const TYPE: u8 = TypeId::TransferEvent as u8;
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

            pub fn command_trb_pointer(&self) -> *const GenericTrb {
                let lo = self.command_trb_pointer_lo() << 4;
                let hi = self.command_trb_pointer_hi();
                (((hi as usize) << 32) | (lo as usize)) as *const GenericTrb
            }
        }
        impl Trb for CommandCompletionEvent {
            const TYPE: u8 = TypeId::CommandCompletionEvent as u8;
        }

        #[repr(C, align(16))]
        pub struct PortStatusChangeEvent {
            data: [u32; 4],
        }
        impl PortStatusChangeEvent {
            getter!(data[0]: u32; 0xFF000000; u8, pub port_id);
        }
        impl Trb for PortStatusChangeEvent {
            const TYPE: u8 = TypeId::PortStatusChangeEvent as u8;
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
        Control = 4,
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
            self.set_dequeue_pointer_lo((((ptr as u64) & 0x00000000FFFFFFFF) >> 4) as u32);
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
    impl From<EndpointId> for DeviceContextIndex {
        fn from(ep_id: EndpointId) -> Self {
            Self(ep_id.address() as usize)
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

    use super::{
        class_driver, descriptor, request_type, EndpointConfig, EndpointId, Request, SetupData,
    };
    use crate::utils::{ArrayMap, ArrayMapError};
    use descriptor::{ConfigurationDescriptor, Descriptor, DeviceDescriptor, InterfaceDescriptor};

    use crate::utils::FixedVec;

    pub struct Device {
        ctx: *const DeviceContext,
        input_ctx: InputContext,
        state: DeviceState,
        doorbell: *mut DoorbellRegister,
        transfer_rings: [Option<TransferRing>; 31],

        is_initialized: bool,
        init_phase: u8,
        buf: [u8; 256],
        num_configurations: u8,
        config_index: u8,
        ep_configs: FixedVec<EndpointConfig, 16>,
        num_ep_configs: usize,
        event_waiters: ArrayMap<SetupData, *mut dyn class_driver::Driver, 4>,
        setup_stage_map: ArrayMap<*const trb::GenericTrb, *const trb::SetupStage, 16>,
    }

    impl Device {
        unsafe fn initialize_ptr(
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
                    transfer_rings_ptr.add(i).write(None);
                }

                let is_initialized_ptr = addr_of_mut!((*ptr).is_initialized);
                is_initialized_ptr.write(false);

                let init_phase_ptr = addr_of_mut!((*ptr).init_phase);
                init_phase_ptr.write(0);

                let buf_ptr: *mut [u8; 256] = addr_of_mut!((*ptr).buf);
                buf_ptr.write_bytes(0, 1);

                let num_configurations_ptr = addr_of_mut!((*ptr).num_configurations);
                num_configurations_ptr.write(0);

                let config_index_ptr = addr_of_mut!((*ptr).config_index);
                config_index_ptr.write(0);

                let ep_configs_ptr = addr_of_mut!((*ptr).ep_configs);
                FixedVec::initialize_ptr(ep_configs_ptr);

                let event_waiters_ptr = addr_of_mut!((*ptr).event_waiters);
                ArrayMap::initialize_ptr(event_waiters_ptr);

                let setup_stage_map_ptr = addr_of_mut!((*ptr).setup_stage_map);
                ArrayMap::initialize_ptr(setup_stage_map_ptr);
            }
            let device = &mut *ptr;

            let slot_ctx = device.input_ctx.enable_slot_context();
            slot_ctx.set_route_string(0);
            slot_ctx.set_root_hub_port_number(port.number());
            slot_ctx.set_context_entries(1);
            slot_ctx.set_speed(port.speed() as u8);

            let ep0_dci = DeviceContextIndex::new_out(0);
            let tr_buf = device.alloc_transfer_ring(ep0_dci, 32)?.buffer_ptr();
            let max_packet_size = port.speed().determine_max_packet_size_for_control_pipe();
            debug!("max_packet_size = {}", max_packet_size);

            let ep0_ctx = device.input_ctx.enable_endpoint(ep0_dci);
            ep0_ctx.set_endpoint_type(EndpointType::Control as u8);
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

        fn port_num(&self) -> u8 {
            unsafe { (*self.ctx).slot_context.root_hub_port_number() }
        }
    }

    impl Device {
        fn is_initialized(&self) -> bool {
            self.is_initialized
        }

        fn start_initialize(&mut self) -> Result<()> {
            self.is_initialized = false;
            self.init_phase = 1;
            self.get_descriptor(
                EndpointId::DEFAULT_CONTROL_PIPE,
                <DeviceDescriptor as Descriptor>::TYPE,
                0,
            )
        }

        fn initialize_phase1(&mut self, transfered_size: usize) -> Result<()> {
            let device_desc = descriptor::from_bytes::<DeviceDescriptor>(&self.buf[..]).unwrap();
            self.num_configurations = device_desc.num_configurations;
            debug!("num_configurations = {}", self.num_configurations);
            self.config_index = 0;
            self.init_phase = 2;
            debug!(
                "issuing Get Config Descriptor: index = {}",
                self.config_index
            );
            self.get_descriptor(
                EndpointId::DEFAULT_CONTROL_PIPE,
                <ConfigurationDescriptor as Descriptor>::TYPE,
                self.config_index,
            )
        }
        fn initialize_phase2(&mut self, transfered_size: usize) -> Result<()> {
            assert!(descriptor::from_bytes::<ConfigurationDescriptor>(&self.buf[..]).is_some());

            struct IfaceIter<'buf> {
                buf: &'buf [u8],
            }
            impl<'buf> IfaceIter<'buf> {
                fn new(conf_desc: &'buf [u8]) -> Self {
                    Self { buf: conf_desc }
                }
            }
            impl<'buf> Iterator for IfaceIter<'buf> {
                type Item = &'buf InterfaceDescriptor;
                fn next(&mut self) -> Option<Self::Item> {
                    loop {
                        let sz = self.buf[0] as usize;
                        if self.buf.len() < sz {
                            return None;
                        }
                        self.buf = &self.buf[sz..];
                        if let Some(desc) = descriptor::from_bytes(self.buf) {
                            return Some(desc);
                        }
                    }
                }
            }

            let device = self as *mut Self; // FIXME: UNSAFE
            let mut class_driver: Option<NonNull<dyn class_driver::Driver + 'static>> = None;
            for if_desc in IfaceIter::new(&self.buf[..transfered_size]) {
                match Self::new_class_driver(device, if_desc) {
                    Ok(driver) => class_driver = Some(driver),
                    Err(Error::UnsupportedInterface) => continue,
                    Err(e) => return Err(e),
                }

                self.ep_configs.clear();
                todo!(); // collect endpoint descriptors
            }

            let conf_desc =
                descriptor::from_bytes::<ConfigurationDescriptor>(&self.buf[..]).unwrap();

            match class_driver {
                None => Ok(()),
                Some(driver) => {
                    self.init_phase = 3;
                    debug!(
                        "issuing Set Configuration: conf_val = {}",
                        conf_desc.configuration_value
                    );
                    let conf_val = conf_desc.configuration_value;
                    self.set_configuration(EndpointId::DEFAULT_CONTROL_PIPE, conf_val)
                }
            }
        }

        fn new_class_driver(
            device: *mut Self,
            if_desc: &InterfaceDescriptor,
        ) -> Result<NonNull<dyn class_driver::Driver + 'static>> {
            let class = if_desc.interface_class;
            let sub = if_desc.interface_sub_class;
            let proto = if_desc.interface_protocol;
            match (class, sub, proto) {
                (3, 1, 1) => {
                    info!("keyboard");
                    unimplemented!();
                }
                (3, 1, 2) => {
                    info!("mouse");
                    let mouse_driver = unsafe {
                        let mouse_driver = MALLOC
                            .lock()
                            .alloc_obj::<class_driver::HidMouseDriver>()
                            .ok_or(Error::NoEnoughMemory)?;
                        (*mouse_driver.as_ptr()).as_mut_ptr().write(
                            class_driver::HidMouseDriver::new(device, if_desc.interface_number),
                        );
                        let tmp = core::mem::transmute::<
                            NonNull<MaybeUninit<class_driver::HidMouseDriver>>,
                            NonNull<class_driver::HidMouseDriver>,
                        >(mouse_driver);
                        NonNull::new_unchecked(&mut *tmp.as_ptr())
                    };
                    Ok(mouse_driver)
                }
                _ => Err(Error::UnsupportedInterface),
            }
        }

        fn on_transfer_event_received(&mut self, trb: &trb::TransferEvent) -> Result<()> {
            if trb.completion_code() != 1 /* Success */ && trb.completion_code() != 13
            /* Short Packet*/
            {
                use trb::Trb;
                debug!(
                    "on_transfer_event_received: invalid trb = {:?}",
                    trb.upcast()
                );
                return Err(Error::TransferFailed);
            }

            let issuer_trb = unsafe { &*trb.trb_pointer() };
            if let Some(normal) = issuer_trb.downcast_ref::<trb::Normal>() {
                unimplemented!();
            }

            let setup_data = match self
                .setup_stage_map
                .remove(&(issuer_trb as *const trb::GenericTrb))
            {
                None => {
                    warn!("No Correspoinding Setup Stage TRB");
                    return Err(Error::NoCorrespondingSetupStage);
                }
                Some((_, setup_stage_trb)) => {
                    let setup_stage_trb = unsafe { &*setup_stage_trb };
                    let mut setup_data = SetupData::default();
                    setup_data.request_type = setup_stage_trb.request_type();
                    setup_data.request = setup_stage_trb.request();
                    setup_data.value = setup_stage_trb.value();
                    setup_data.index = setup_stage_trb.index();
                    setup_data.length = setup_stage_trb.length();
                    setup_data
                }
            };

            let transfered_size;

            if let Some(data_stage) = issuer_trb.downcast_ref::<trb::DataStage>() {
                let residual_bytes = trb.trb_transfer_length() as usize;
                debug!("residual_bytes = {}", residual_bytes);
                transfered_size = data_stage.trb_transfer_length() as usize - residual_bytes;
            } else if let Some(status_stage) = issuer_trb.downcast_ref::<trb::StatusStage>() {
                // TODO: update max packet size
                transfered_size = 0;
            } else {
                unimplemented!();
            }

            self.on_control_completed(
                EndpointId {
                    addr: trb.endpoint_id(),
                },
                setup_data,
                transfered_size,
            )
        }

        fn on_control_completed(
            &mut self,
            ep_id: EndpointId,
            setup_data: SetupData,
            transfered_size: usize,
        ) -> Result<()> {
            debug!(
                "device::on_control_completed: transfered_size = {}, dir = {}",
                transfered_size,
                setup_data.direction(),
            );

            if self.is_initialized {
                match self.event_waiters.get_mut(&setup_data) {
                    Some(w) => {
                        return unsafe {
                            (**w).on_control_completed(
                                ep_id,
                                setup_data,
                                &self.buf[..transfered_size],
                            )
                        };
                    }
                    None => {
                        return Err(Error::NoWaiter);
                    }
                }
            }

            let bytes = &self.buf[..transfered_size];
            match self.init_phase {
                1 => {
                    if setup_data.request == Request::GetDescriptor as u8
                        && descriptor::from_bytes::<DeviceDescriptor>(bytes).is_some()
                    {
                        self.initialize_phase1(transfered_size)
                    } else {
                        warn!("failed to go initialize_phase1");
                        Err(Error::InvalidPhase)
                    }
                }
                2 => {
                    if setup_data.request == Request::GetDescriptor as u8
                        && descriptor::from_bytes::<ConfigurationDescriptor>(bytes).is_some()
                    {
                        self.initialize_phase2(transfered_size)
                    } else {
                        warn!("failed to go initialize_phase2");
                        Err(Error::InvalidPhase)
                    }
                }
                3 => {
                    unimplemented!();
                }
                _ => unimplemented!(),
            }
        }

        fn control_in(
            &mut self,
            ep_id: EndpointId,
            setup_data: SetupData,
            issuer: Option<&mut (dyn super::class_driver::Driver + 'static)>,
            use_buf: bool,
        ) -> Result<()> {
            if let Some(issuer) = issuer {
                self.event_waiters
                    .insert(setup_data.clone(), issuer)
                    .map_err(|e| match e {
                        ArrayMapError::NoSpace => Error::TooManyWaiters,
                        ArrayMapError::SameKeyRegistered => {
                            panic!("same setup_data registered")
                        }
                    })?;
            }

            debug!("Device::control_in: ep_id = {}", ep_id.address());
            if 15 < ep_id.number() {
                return Err(Error::InvalidEndpointNumber);
            }

            let dci = DeviceContextIndex::from(ep_id);
            let tr = self.transfer_rings[dci.0 - 1]
                .as_mut()
                .ok_or(Error::TransferRingNotSet)?;

            if use_buf {
                // TODO: update max packet size

                let setup_stage = trb::SetupStage::new_in_data_stage(setup_data);
                let setup_trb_ptr = tr.push(&setup_stage);
                let setup_trb_ptr = setup_trb_ptr.downcast_ref::<trb::SetupStage>().unwrap()
                    as *const trb::SetupStage;

                let mut data = trb::DataStage::new_in(&self.buf[..]);
                data.set_interrupt_on_completion(1);
                let data_trb_ptr = tr.push(&data) as *const trb::GenericTrb;

                let status = trb::StatusStage::default();
                let status_stage_trb = tr.push(&status);
                debug!("status_stage_trb = {:p}", status_stage_trb);

                self.setup_stage_map
                    .insert(data_trb_ptr, setup_trb_ptr)
                    .map_err(|e| match e {
                        ArrayMapError::NoSpace => Error::TooManyWaiters,
                        ArrayMapError::SameKeyRegistered => {
                            panic!("same data_stage_trb_ptr registered")
                        }
                    })?;

                unsafe { (*self.doorbell).ring(dci.0 as u8, 0) };
            } else {
                unimplemented!();
            }

            Ok(())
        }

        fn control_out(
            &mut self,
            ep_id: EndpointId,
            setup_data: SetupData,
            issuer: Option<&mut dyn super::class_driver::Driver>,
            use_buf: bool,
        ) -> Result<()> {
            unimplemented!();
        }

        fn get_descriptor(
            &mut self,
            ep_id: EndpointId,
            desc_type: u8,
            desc_index: u8,
        ) -> Result<()> {
            let mut setup_data = SetupData::default();
            setup_data.set_direction(request_type::Direcrion::DeviceToHost as u8);
            setup_data.set_typ(request_type::Type::Standard as u8);
            setup_data.set_recipient(request_type::Recipient::Device as u8);
            setup_data.request = Request::GetDescriptor as u8;
            setup_data.value = ((desc_type as u16) << 8) | (desc_index as u16);
            setup_data.index = 0;
            setup_data.length = self.buf.len() as u16;

            self.control_in(ep_id, setup_data, None, true)
        }

        fn set_configuration(&mut self, ep_id: EndpointId, config_value: u8) -> Result<()> {
            unimplemented!()
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

        pub fn find_by_slot(&self, slot_id: u8) -> Option<&Device> {
            self.devices.get(slot_id as usize).copied().and_then(|p| {
                if p.is_null() {
                    None
                } else {
                    Some(unsafe { &*p })
                }
            })
        }

        pub fn find_by_slot_mut(&mut self, slot_id: u8) -> Option<&mut Device> {
            self.devices
                .get_mut(slot_id as usize)
                .copied()
                .and_then(|p| {
                    if p.is_null() {
                        None
                    } else {
                        Some(unsafe { &mut *p })
                    }
                })
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

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
            debug!("reset done on port {}", self.port_num);
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

mod request_type {
    #[repr(u8)]
    pub enum Recipient {
        Device = 0,
        Interface = 1,
        Endpoint = 2,
        Other = 3,
    }
    #[repr(u8)]
    pub enum Type {
        Standard = 0,
        Class = 1,
        Vendor = 2,
    }
    #[repr(u8)]
    pub enum Direcrion {
        HostToDevice = 0,
        DeviceToHost = 1,
    }
}

#[repr(C)]
enum Request {
    GetDescriptor = 6,
}

#[derive(Default, Clone, PartialEq, Eq)]
#[repr(C)]
pub struct SetupData {
    pub request_type: u8,
    request: u8,
    value: u16,
    index: u16,
    length: u16,
}
impl SetupData {
    getter!(request_type: u8; 0b00011111; u8, pub recipient);
    setter!(request_type: u8; 0b00011111; u8, pub set_recipient);

    getter!(request_type: u8; 0b01100000; u8, pub typ);
    setter!(request_type: u8; 0b01100000; u8, pub set_typ);

    getter!(request_type: u8; 0b10000000; u8, pub direction);
    setter!(request_type: u8; 0b10000000; u8, pub set_direction);
}

#[repr(u8)]
pub enum EndpointType {
    Control = 0,
    Isochronous = 1,
    Bulk = 2,
    Interrupt = 3,
}

#[derive(Clone, Copy)]
pub struct EndpointId {
    addr: u8,
}
impl EndpointId {
    const DEFAULT_CONTROL_PIPE: EndpointId = EndpointId { addr: 1 };

    pub fn new_in(ep_num: u8) -> Self {
        Self {
            addr: (ep_num << 1) | 1,
        }
    }

    pub fn new_out(ep_num: u8) -> Self {
        Self {
            addr: (ep_num << 1) | (ep_num == 0) as u8,
        }
    }

    pub fn address(&self) -> u8 {
        self.addr
    }

    pub fn number(&self) -> u8 {
        self.addr >> 1
    }

    pub fn is_in(&self) -> bool {
        (self.addr & 1) == 1
    }

    pub fn is_out(&self) -> bool {
        (self.addr & 1) == 0
    }
}

pub struct EndpointConfig {
    ep_id: EndpointId,
    ep_type: EndpointType,
    max_packet_size: usize,
    interval: usize,
}

mod descriptor {
    #[repr(u8)]
    pub enum Type {
        Device = 1,
        Configuration = 2,
        Interface = 4,
    }

    pub trait Descriptor {
        const TYPE: u8;
    }

    pub fn from_bytes<D: Descriptor>(bytes: &[u8]) -> Option<&D> {
        if bytes[0] == core::mem::size_of::<D>() as u8 && bytes[1] == D::TYPE {
            let p = bytes.as_ptr() as *const D;
            Some(unsafe { &*p })
        } else {
            None
        }
    }

    #[repr(C, packed)]
    pub struct DeviceDescriptor {
        pub length: u8,
        pub descriptor_type: u8,
        pub usb_release: u16,
        pub device_class: u8,
        pub device_sub_class: u8,
        pub device_protocol: u8,
        pub max_packet_size: u8,
        pub vendor_id: u16,
        pub product_id: u16,
        pub device_release: u16,
        pub manifacturer: u8,
        pub product: u8,
        pub serial_number: u8,
        pub num_configurations: u8,
    }
    impl Descriptor for DeviceDescriptor {
        const TYPE: u8 = Type::Device as u8;
    }

    #[repr(C, packed)]
    pub struct ConfigurationDescriptor {
        pub length: u8,
        pub descriptor_type: u8,
        pub total_length: u16,
        pub num_interfaces: u8,
        pub configuration_value: u8,
        pub configuration_id: u8,
        pub attributes: u8,
        pub max_power: u8,
    }
    impl Descriptor for ConfigurationDescriptor {
        const TYPE: u8 = Type::Configuration as u8;
    }

    #[repr(C, packed)]
    pub struct InterfaceDescriptor {
        pub length: u8,
        pub descriptor_type: u8,
        pub interface_number: u8,
        pub alternate_setting: u8,
        pub num_endpoints: u8,
        pub interface_class: u8,
        pub interface_sub_class: u8,
        pub interface_protocol: u8,
        pub interface_id: u8,
    }
    impl Descriptor for InterfaceDescriptor {
        const TYPE: u8 = Type::Interface as u8;
    }
}

pub mod class_driver {
    use super::Result;
    use super::{EndpointConfig, EndpointId, SetupData};

    pub trait Driver {
        fn initialize(&mut self) -> Result<()>;
        fn set_endpoint(&mut self, config: &EndpointConfig) -> Result<()>;
        fn on_control_completed(
            &mut self,
            ep_id: EndpointId,
            setup_data: SetupData,
            buf: &[u8],
        ) -> Result<()>;
        fn on_interrupt_completed(&mut self, ep_id: EndpointId, buf: &[u8]) -> Result<()>;
    }

    pub trait HidDriver: Driver {}

    pub struct HidMouseDriver {
        device: *mut super::xhci::Device,
        interface_idx: u8,
        in_packet_size: usize,
    }
    impl Driver for HidMouseDriver {
        fn initialize(&mut self) -> Result<()> {
            unimplemented!()
        }
        fn set_endpoint(&mut self, config: &EndpointConfig) -> Result<()> {
            unimplemented!()
        }
        fn on_control_completed(
            &mut self,
            ep_id: EndpointId,
            setup_data: SetupData,
            buf: &[u8],
        ) -> Result<()> {
            unimplemented!()
        }
        fn on_interrupt_completed(&mut self, ep_id: EndpointId, buf: &[u8]) -> Result<()> {
            unimplemented!()
        }
    }
    impl HidDriver for HidMouseDriver {}
    impl HidMouseDriver {
        pub fn new(device: *mut super::xhci::Device, interface_idx: u8) -> Self {
            Self {
                device,
                interface_idx,
                in_packet_size: 3,
            }
        }
    }

    // pub struct HidKeyboardDriver {}
    // impl Driver for HidKeyboardDriver {}
    // impl HidDriver for HidKeyboardDriver {}
}
