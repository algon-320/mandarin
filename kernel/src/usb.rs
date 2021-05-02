#![allow(clippy::transmute_ptr_to_ptr)]
#![allow(clippy::identity_op)]

#[derive(Debug)]
pub enum Error {
    NoEnoughMemory,
    InvalidPhase,
    InvalidSlotId,
    InvalidEndpointNumber,
    InvalidEndpointType { ty: u8 },
    DeviceAlreadyAllocated,
    TransferRingNotSet,
    TransferFailed { slot_id: u8 },
    NoCorrespondingSetupStage,
    NoWaiter,
    TooManyWaiters,
    UnsupportedInterface,
}
pub type Result<T> = core::result::Result<T, Error>;

use crate::sync::spin::SpinMutex;
use crate::utils::StaticMallocator;

const MALLOC_BUF_SIZE: usize = 4 * 1024 * 1024; // 4MiB
static MALLOC: SpinMutex<StaticMallocator<MALLOC_BUF_SIZE>> =
    SpinMutex::new("usb/malloc", StaticMallocator::new());

pub mod xhci {
    use core::mem::{size_of, MaybeUninit};
    use core::ptr::{addr_of_mut, null, null_mut, NonNull};

    use super::MALLOC;
    use super::{Error, Result};
    use crate::utils::{ArrayMap, ArrayMapError, Buffer, FixedVec};
    use trb::Trb;

    #[repr(transparent)]
    struct MemMapped<T>(T);
    impl<T> MemMapped<T> {
        pub fn read(&self) -> T {
            // Safety: the representation of `MemMapped<T>` is the same as that of `T`,
            unsafe { (self as *const Self as *const T).read_volatile() }
        }
        pub fn write(&mut self, val: T) {
            // Safety: the representation of `MemMapped<T>` is the same as that of `T`,
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
            // RO
            getter!(data: u32; 0x000000FF; u8, pub max_device_slots);

            // RO
            getter!(data: u32; 0xFF000000; u8, pub max_ports);
        }

        #[repr(C)]
        pub struct Hcsparams2 {
            data: u32,
        }
        impl Hcsparams2 {
            // RO
            getter!(data: u32; 0x03E00000; u8, max_scratchpad_bufs_hi);
            getter!(data: u32; 0xF8000000; u8, max_scratchpad_bufs_lo);

            pub fn max_scratchpad_buf(&self) -> usize {
                let lo = self.max_scratchpad_bufs_lo() as usize;
                let hi = self.max_scratchpad_bufs_hi() as usize;
                (hi << 5) | lo
            }
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
            // RO
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
            getter!(data: u64; 0x0000000000000001;  u8, pub ring_cycle_state);
            setter!(data: u64; 0x0000000000000001;  u8, pub set_ring_cycle_state);

            // RW1S
            getter!(data: u64; 0x0000000000000002;  u8, pub command_stop);
            setter!(data: u64; 0x0000000000000002;  u8, pub set_command_stop);

            // RW1S
            getter!(data: u64; 0x0000000000000004;  u8, pub command_abort);
            setter!(data: u64; 0x0000000000000004;  u8, pub set_command_abort);

            getter!(data: u64; 0xFFFFFFFFFFFFFFC0; u64, command_ring_pointer);
            setter!(data: u64; 0xFFFFFFFFFFFFFFC0; u64, set_command_ring_pointer);

            pub fn pointer(&self) -> usize {
                (self.command_ring_pointer() as usize) << 6
            }
            pub fn set_pointer(&mut self, ptr: usize) {
                debug_assert!(ptr & 0x3F == 0);
                let ptr = ((ptr & 0xFFFFFFFFFFFFFFC0) >> 6) as u64;
                self.set_command_ring_pointer(ptr);
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
                (self.device_context_base_address_array_pointer() as usize) << 6
            }
            pub fn set_pointer(&mut self, ptr: usize) {
                debug_assert!(ptr & 0x3F == 0);
                let ptr = ((ptr & 0xFFFFFFFFFFFFFFC0) >> 6) as u64;
                self.set_device_context_base_address_array_pointer(ptr);
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
        pub fn ring(&mut self, target: u8) {
            self.ring_with_stream_id(target, 0)
        }
        pub fn ring_with_stream_id(&mut self, target: u8, stream_id: u16) {
            self.reg.modify_with(|reg| {
                reg.set_db_target(target);
                reg.set_db_stream_id(stream_id);
            })
        }
    }

    pub struct Controller {
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
        const DEVICES_SIZE: usize = 16;

        /// Safety: `mmio_base` muse be a valid base address of the MMIO space.
        pub unsafe fn new(mmio_base: usize) -> Result<Self> {
            let cap_regs = mmio_base as *mut CapabilityRegisters;
            let max_ports = (*cap_regs).hcsparams1.read().max_ports();

            let dboff = (*cap_regs).dboff.read().offset();
            let doorbell_zero = (mmio_base + dboff) as *mut DoorbellRegister;

            let rtsoff = (*cap_regs).rtsoff.read().offset();
            let primary_interrupter = (mmio_base + rtsoff + 0x20) as *mut InterrupterRegisterSet;

            let caplength = (*cap_regs).caplength.read();
            let op_regs = (mmio_base + caplength as usize) as *mut OperationalRegisters;

            if (*op_regs).usbsts.read().host_controller_halted() == 0 {
                (*op_regs).usbcmd.modify_with(|usbcmd| {
                    usbcmd.set_run_stop(0);
                });
            }

            // Host controller must be halted
            while (*op_regs).usbsts.read().host_controller_halted() == 0 {}
            debug!("host controller halted");

            // Update allocation boundary with PAGESIZE
            let page_size = (*op_regs).pagesize.read().page_size();
            MALLOC.lock().default_boundary = page_size;

            Self::request_hc_ownership(mmio_base, cap_regs);

            // Reset the controller
            {
                (*op_regs).usbcmd.modify_with(|usbcmd| {
                    usbcmd.set_host_controller_reset(1);
                });
                while (*op_regs).usbcmd.read().host_controller_reset() != 0 {}
                while (*op_regs).usbsts.read().controller_not_ready() != 0 {}
            }

            let max_slots = (*cap_regs).hcsparams1.read().max_device_slots();
            let slots = core::cmp::min(max_slots, Self::DEVICES_SIZE as u8);
            debug!("up to {} slots", slots);

            // Set "Max Slots Enabled" field in CONFIG
            (*op_regs).config.modify_with(|config| {
                config.set_max_device_slots_enabled(slots);
            });

            let max_scratchpad_buffer_pages = (*cap_regs).hcsparams2.read().max_scratchpad_buf();
            let scratchpad_buffer_array_ptr = if max_scratchpad_buffer_pages > 0 {
                debug!(
                    "max scratchpad buffer: {} pages",
                    max_scratchpad_buffer_pages
                );

                let mut malloc = MALLOC.lock();

                #[allow(unused_unsafe)]
                let buf_arr: &mut [MaybeUninit<*const u8>] = unsafe {
                    malloc
                        .alloc_slice_ext::<*const u8>(
                            max_scratchpad_buffer_pages,
                            64,
                            Some(page_size),
                        )
                        .ok_or(Error::NoEnoughMemory)?
                        .as_mut()
                };

                for ptr in buf_arr.iter_mut() {
                    #[allow(unused_unsafe)]
                    let buf: &mut [u8] = unsafe {
                        malloc
                            .alloc(page_size, page_size, Some(page_size))
                            .ok_or(Error::NoEnoughMemory)?
                            .as_mut()
                    };
                    *ptr = MaybeUninit::new(buf.as_ptr());
                }

                // Safety: `buf_arr` has been properly initialized.
                #[allow(unused_unsafe)]
                let buf_arr = unsafe {
                    core::mem::transmute::<&mut [MaybeUninit<*const u8>], &mut [*const u8]>(buf_arr)
                };
                buf_arr.as_ptr()
            } else {
                null()
            };

            let devmgr = DeviceManager::new(
                slots as usize,
                doorbell_zero.add(1),
                scratchpad_buffer_array_ptr,
            )?;

            let mut dcbaap = bitmap::Dcbaap::default();
            let device_contexts = devmgr.dcbaap();
            dcbaap.set_pointer(device_contexts as usize);
            (*op_regs).dcbaap.write(dcbaap);

            let cr = CommandRing::with_capacity(32)?;
            // register the address of the Command Ring buffer
            (*op_regs).crcr.modify_with(|value| {
                value.set_ring_cycle_state(cr.cycle_bit as u8);
                value.set_command_stop(0);
                value.set_command_abort(0);
                value.set_pointer(cr.buffer_ptr() as usize);
            });

            let mut er = EventRing::with_capacity(32)?;
            er.initialize(primary_interrupter);

            // Enable interrupt for the primary interrupter
            (*primary_interrupter).iman.modify_with(|iman| {
                iman.set_interrupt_pending(1);
                iman.set_interrupt_enable(1);
            });

            // Enable interrupt for the controller
            (*op_regs).usbcmd.modify_with(|usbcmd| {
                usbcmd.set_interrupter_enable(1);
            });

            // allocate ports
            let ports = {
                let port_regs_base = ((op_regs as usize) + 0x400) as *mut PortRegisterSet;

                #[allow(unused_unsafe)]
                let ports: &mut [MaybeUninit<Port>] = unsafe {
                    MALLOC
                        .lock()
                        .alloc_slice::<Port>((max_ports + 1) as usize)
                        .ok_or(Error::NoEnoughMemory)?
                        .as_mut()
                };

                ports[0] = MaybeUninit::new(Port::new(0, null_mut()));
                for port_num in 1..=max_ports {
                    let port_regs = port_regs_base.add((port_num - 1) as usize);
                    ports[port_num as usize] = MaybeUninit::new(Port::new(port_num, port_regs));
                }

                // Safety: `ports` has been properly initialized.
                #[allow(unused_unsafe)]
                unsafe {
                    core::mem::transmute::<&mut [MaybeUninit<Port>], &mut [Port]>(ports)
                }
            };

            Ok(Self {
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

        fn request_hc_ownership(mmio_base: usize, cap_regs: *mut CapabilityRegisters) {
            type MmExtendedReg = MemMapped<bitmap::ExtendedRegister>;

            fn next(current: *mut MmExtendedReg, step: usize) -> *mut MmExtendedReg {
                if step == 0 {
                    null_mut()
                } else {
                    current.wrapping_add(step as usize)
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

        fn ring_doorbell(doorbell: *mut DoorbellRegister) {
            trace!("ring the doorbell zero (Command Ring)");
            unsafe { (*doorbell).ring(0) };
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
                self.cr.push(cmd.upcast());
                Self::ring_doorbell(self.doorbell_zero);
            }
            Ok(())
        }

        fn address_device(&mut self, port_num: u8, slot_id: u8) -> Result<()> {
            trace!("address_device: port = {}, slot = {}", port_num, slot_id);
            let port = &self.ports[port_num as usize];
            let input_ctx = self.devmgr.add_device(port, slot_id)?;

            self.ports[port_num as usize].set_config_phase(PortConfigPhase::AddressingDevice);
            let mut cmd = trb::AddressDeviceCommand::default();
            cmd.set_input_context_ptr(input_ctx);
            cmd.set_slot_id(slot_id);
            self.cr.push(cmd.upcast());
            Self::ring_doorbell(self.doorbell_zero);

            Ok(())
        }

        pub fn process_event(&mut self) -> Result<()> {
            if let Some(trb) = self.er.front() {
                trace!("event found: TRB type = {}", trb.trb_type());

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

            if let Some(cmd_trb) = dev.command_trb.take() {
                debug!("command TRB found");
                self.cr.push(&cmd_trb);
                Self::ring_doorbell(self.doorbell_zero);
            }

            if dev.is_initialized()
                && self.ports[dev.port_num() as usize].config_phase()
                    == PortConfigPhase::InitializingDevice
            {
                let input_ctx = dev.configure_endpoints()?;

                self.ports[dev.port_num() as usize]
                    .set_config_phase(PortConfigPhase::ConfiguringEndpoints);
                let mut cmd = trb::ConfigureEndpointCommand::default();
                cmd.set_input_context_ptr(input_ctx);
                cmd.set_slot_id(slot_id);
                self.cr.push(cmd.upcast());
                Self::ring_doorbell(self.doorbell_zero);
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

            match issuer_type {
                trb::EnableSlotCommand::TYPE => match self.addressing_port {
                    Some(port_num)
                        if self.ports[port_num as usize].config_phase()
                            == PortConfigPhase::EnablingSlot =>
                    {
                        self.address_device(port_num, slot_id)
                    }
                    _ => {
                        warn!("addressing_port is None");
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
                                "addressing_port = {:?}, but the event is on port = {}",
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
                        trace!("looking for the next port to address ...");
                        for i in 1..=self.max_ports {
                            if self.ports[i as usize].config_phase()
                                == PortConfigPhase::WaitingAddressed
                            {
                                trace!("the next port is port {}!", i);
                                self.reset_port(i)?;
                                break;
                            }
                        }
                        let dev = self
                            .devmgr
                            .find_by_slot_mut(slot_id)
                            .ok_or(Error::InvalidSlotId)?;
                        let port_num = dev.port_num();
                        self.ports[port_num as usize]
                            .set_config_phase(PortConfigPhase::InitializingDevice);
                        dev.on_command_completion_event_received(issuer_type)?;
                        if let Some(cmd_trb) = dev.command_trb.take() {
                            debug!("command TRB found");
                            self.cr.push(&cmd_trb);
                            Self::ring_doorbell(self.doorbell_zero);
                        }
                        Ok(())
                    }
                }
                trb::EvaluateContextCommand::TYPE => {
                    let dev = self
                        .devmgr
                        .find_by_slot_mut(slot_id)
                        .ok_or(Error::InvalidSlotId)?;
                    let port_num = dev.port_num();
                    if self.ports[port_num as usize].config_phase()
                        != PortConfigPhase::InitializingDevice
                    {
                        Err(Error::InvalidPhase)
                    } else {
                        dev.on_command_completion_event_received(issuer_type)?;
                        if let Some(cmd_trb) = dev.command_trb.take() {
                            debug!("command TRB found");
                            self.cr.push(&cmd_trb);
                            Self::ring_doorbell(self.doorbell_zero);
                        }
                        Ok(())
                    }
                }
                trb::ConfigureEndpointCommand::TYPE => {
                    let dev = self
                        .devmgr
                        .find_by_slot_mut(slot_id)
                        .ok_or(Error::InvalidSlotId)?;
                    let port_num = dev.port_num();

                    if self.ports[port_num as usize].config_phase()
                        != PortConfigPhase::ConfiguringEndpoints
                    {
                        Err(Error::InvalidPhase)
                    } else {
                        dev.on_endpoints_configured()?;
                        self.ports[port_num as usize].set_config_phase(PortConfigPhase::Configured);
                        Ok(())
                    }
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
            let port = &mut self.ports[port_id as usize];
            trace!(
                "PortStatusChangeEvent: port_id = {}, phase = {:?}, (bits = {:032b})",
                port_id,
                port.config_phase(),
                port.bits(),
            );
            match port.config_phase() {
                PortConfigPhase::NotConnected => {
                    if port.is_connect_status_changed() {
                        port.clear_connect_status_change();
                        self.reset_port(port_id)
                    } else {
                        trace!("skipping reset_port: port_id = {}", port_id);
                        Ok(())
                    }
                }
                PortConfigPhase::ResettingPort => {
                    if port.is_port_reset_changed() {
                        self.enable_slot(port_id)
                    } else {
                        trace!("skipping: enable_slot: port_id = {}", port_id);
                        Ok(())
                    }
                }
                PortConfigPhase::EnablingSlot => {
                    trace!("skipping: port_id = {}", port_id);
                    Ok(())
                }
                PortConfigPhase::WaitingAddressed => {
                    trace!("waiting addressed: port_id = {}", port_id);
                    Ok(())
                }
                phase => {
                    warn!(
                        "config_phase = {:?} (should be {:?}, {:?}, {:?}, or {:?})",
                        phase,
                        PortConfigPhase::NotConnected,
                        PortConfigPhase::ResettingPort,
                        PortConfigPhase::EnablingSlot,
                        PortConfigPhase::WaitingAddressed,
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
            let buf: &mut [MaybeUninit<GenericTrb>] = unsafe {
                MALLOC
                    .lock()
                    .alloc_slice_ext::<GenericTrb>(buf_size, 64, Some(64 * 1024))
                    .ok_or(Error::NoEnoughMemory)?
                    .as_mut()
            };
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

        pub fn push(&mut self, trb: &trb::GenericTrb) -> &GenericTrb {
            self.copy_to_last(trb.clone());
            let written_idx = self.write_idx;
            self.write_idx += 1;

            if self.write_idx + 1 == self.buf.len() {
                let mut link = trb::Link::new(self.buffer_ptr() as usize);
                link.set_toggle_cycle(1);
                let trb = <trb::Link as trb::Trb>::upcast(&link);
                self.copy_to_last(trb.clone());
                self.write_idx = 0;
                self.cycle_bit = !self.cycle_bit;
            }
            trace!("TRB (type: {}) pushed: {:?}", trb.trb_type(), trb);
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

            let buf: &mut [MaybeUninit<GenericTrb>] = unsafe {
                malloc
                    .alloc_slice_ext::<GenericTrb>(buf_size, 64, Some(64 * 1024))
                    .ok_or(Error::NoEnoughMemory)?
                    .as_mut()
            };
            for p in buf.iter_mut() {
                *p = MaybeUninit::zeroed();
            }
            let buf = unsafe {
                core::mem::transmute::<&mut [MaybeUninit<GenericTrb>], &mut [GenericTrb]>(buf)
            };
            let buf = buf as *const [GenericTrb];

            let table: &mut [MaybeUninit<EventRingSegmentTableEntry>] = unsafe {
                malloc
                    .alloc_slice_ext::<EventRingSegmentTableEntry>(1, 64, Some(64 * 1024))
                    .ok_or(Error::NoEnoughMemory)?
                    .as_mut()
            };
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
            let table = table as *const [EventRingSegmentTableEntry];

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
        use super::{EndpointId, InputContext, SetupData};

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
            EvaluateContextCommand = 13,

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
        impl Normal {
            getter!(data[0]: u32; 0xFFFFFFFF; u32, data_buffer_lo);
            setter!(data[0]: u32; 0xFFFFFFFF; u32, set_data_buffer_lo);
            getter!(data[1]: u32; 0xFFFFFFFF; u32, data_buffer_hi);
            setter!(data[1]: u32; 0xFFFFFFFF; u32, set_data_buffer_hi);

            getter!(data[2]: u32; 0x0001FFFF; u32, pub trb_transfer_length);
            setter!(data[2]: u32; 0x0001FFFF; u32, pub set_trb_transfer_length);

            getter!(data[2]: u32; 0x003E0000; u32, pub td_size);
            setter!(data[2]: u32; 0x003E0000; u32, pub set_td_size);

            getter!(data[3]: u32; 0x00000004;  u8, pub interrupt_on_short_packet);
            setter!(data[3]: u32; 0x00000004;  u8, pub set_interrupt_on_short_packet);

            getter!(data[3]: u32; 0x00000020;  u8, pub interrupt_on_completion);
            setter!(data[3]: u32; 0x00000020;  u8, pub set_interrupt_on_completion);

            setter!(data[3]: u32; 0x0000FC00;  u8, set_trb_type);

            pub fn data_buffer(&self) -> *mut u8 {
                (((self.data_buffer_hi() as usize) << 32) | (self.data_buffer_lo() as usize))
                    as *mut u8
            }
            pub fn set_data_buffer(&mut self, ptr: *mut u8) {
                let ptr = ptr as usize;
                self.set_data_buffer_hi(((ptr & 0xFFFFFFFF00000000) >> 32) as u32);
                self.set_data_buffer_lo(((ptr & 0x00000000FFFFFFFF) >> 00) as u32);
            }
        }
        impl Default for Normal {
            fn default() -> Self {
                let mut trb = Self { data: [0; 4] };
                trb.set_trb_type(Self::TYPE);
                trb
            }
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

            fn new(setup_data: SetupData, transfer_type: u8) -> Self {
                let mut setup = Self::default();
                setup.set_request_type(setup_data.request_type);
                setup.set_request(setup_data.request);
                setup.set_value(setup_data.value);
                setup.set_index(setup_data.index);
                setup.set_length(setup_data.length);
                setup.set_transfer_type(transfer_type);
                setup
            }
            pub fn new_no_data_stage(setup_data: SetupData) -> Self {
                Self::new(setup_data, 0)
            }
            pub fn new_out_data_stage(setup_data: SetupData) -> Self {
                Self::new(setup_data, 2)
            }
            pub fn new_in_data_stage(setup_data: SetupData) -> Self {
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

            pub fn data_buffer(&self) -> *mut u8 {
                (((self.data_buffer_hi() as usize) << 32) | (self.data_buffer_lo() as usize))
                    as *mut u8
            }
            pub fn set_data_buffer(&mut self, ptr: *mut u8) {
                let ptr = ptr as usize;
                self.set_data_buffer_hi(((ptr & 0xFFFFFFFF00000000) >> 32) as u32);
                self.set_data_buffer_lo(((ptr & 0x00000000FFFFFFFF) >> 00) as u32);
            }

            fn new(buf: *mut u8, len: usize, dir_in: bool) -> Self {
                let mut trb = Self::default();
                trb.set_data_buffer(buf);
                trb.set_trb_transfer_length(len as u32);
                trb.set_td_size(0);
                trb.set_direction(dir_in as u8);
                trb
            }
            pub fn new_out(buf: *const u8, len: usize) -> Self {
                Self::new(buf as *mut u8, len, false)
            }
            pub fn new_in(buf: *mut u8, len: usize) -> Self {
                Self::new(buf, len, true)
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
            getter!(data[0]: u32; 0xFFFFFFF0; u32, ring_segment_pointer_lo);
            setter!(data[0]: u32; 0xFFFFFFF0; u32, set_ring_segment_pointer_lo);
            getter!(data[1]: u32; 0xFFFFFFFF; u32, ring_segment_pointer_hi);
            setter!(data[1]: u32; 0xFFFFFFFF; u32, set_ring_segment_pointer_hi);

            setter!(data[3]: u32; 0x00000002; u8, pub set_toggle_cycle);
            setter!(data[3]: u32; 0x0000FC00; u8, set_trb_type);

            pub fn ring_segment_pointer(&self) -> usize {
                let lo = (self.ring_segment_pointer_lo() as usize) << 4;
                let hi = (self.ring_segment_pointer_hi() as usize) << 32;
                hi | lo
            }
            pub fn set_ring_segment_pointer(&mut self, ptr: usize) {
                debug_assert!(ptr & 0xF == 0);
                let lo = ((ptr & 0x00000000FFFFFFFF) >> 4) as u32;
                let hi = ((ptr & 0xFFFFFFFF00000000) >> 32) as u32;
                self.set_ring_segment_pointer_lo(lo);
                self.set_ring_segment_pointer_hi(hi);
            }

            pub fn new(next_ring_segment_ptr: usize) -> Self {
                let mut trb = Self::default();
                trb.set_ring_segment_pointer(next_ring_segment_ptr);
                trb
            }
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

            pub(super) fn input_context_ptr(&self) -> *const InputContext {
                (((self.input_ctx_ptr_hi() as u64) << 32) | ((self.input_ctx_ptr_lo() << 4) as u64))
                    as usize as *const InputContext
            }
            pub(super) fn set_input_context_ptr(&mut self, ptr: *const InputContext) {
                let ptr = ptr as usize as u64;
                debug_assert!(ptr & 0xF == 0);
                self.set_input_ctx_ptr_lo(((ptr & 0x00000000FFFFFFFF) >> 4) as u32);
                self.set_input_ctx_ptr_hi(((ptr & 0xFFFFFFFF00000000) >> 32) as u32);
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
            getter!(data[0]: u32; 0xFFFFFFF0; u32, input_ctx_ptr_lo);
            setter!(data[0]: u32; 0xFFFFFFF0; u32, set_input_ctx_ptr_lo);
            getter!(data[1]: u32; 0xFFFFFFFF; u32, input_ctx_ptr_hi);
            setter!(data[1]: u32; 0xFFFFFFFF; u32, set_input_ctx_ptr_hi);

            setter!(data[3]: u32; 0x0000FC00;  u8, set_trb_type);

            getter!(data[3]: u32; 0xFF000000;  u8, pub slot_id);
            setter!(data[3]: u32; 0xFF000000;  u8, pub set_slot_id);

            pub(super) fn input_context_ptr(&self) -> *const InputContext {
                (((self.input_ctx_ptr_hi() as u64) << 32) | ((self.input_ctx_ptr_lo() << 4) as u64))
                    as usize as *const InputContext
            }
            pub(super) fn set_input_context_ptr(&mut self, ptr: *const InputContext) {
                let ptr = ptr as usize as u64;
                debug_assert!(ptr & 0xF == 0);
                self.set_input_ctx_ptr_lo(((ptr & 0x00000000FFFFFFFF) >> 4) as u32);
                self.set_input_ctx_ptr_hi(((ptr & 0xFFFFFFFF00000000) >> 32) as u32);
            }
        }
        impl Default for ConfigureEndpointCommand {
            fn default() -> Self {
                let mut trb = Self { data: [0; 4] };
                trb.set_trb_type(Self::TYPE);
                trb
            }
        }
        impl Trb for ConfigureEndpointCommand {
            const TYPE: u8 = TypeId::ConfigureEndpointCommand as u8;
        }

        #[repr(C, align(16))]
        pub struct EvaluateContextCommand {
            data: [u32; 4],
        }
        impl EvaluateContextCommand {
            getter!(data[0]: u32; 0xFFFFFFF0; u32, input_ctx_ptr_lo);
            setter!(data[0]: u32; 0xFFFFFFF0; u32, set_input_ctx_ptr_lo);
            getter!(data[1]: u32; 0xFFFFFFFF; u32, input_ctx_ptr_hi);
            setter!(data[1]: u32; 0xFFFFFFFF; u32, set_input_ctx_ptr_hi);

            setter!(data[3]: u32; 0x0000FC00;  u8, set_trb_type);

            getter!(data[3]: u32; 0xFF000000;  u8, pub slot_id);
            setter!(data[3]: u32; 0xFF000000;  u8, pub set_slot_id);

            pub(super) fn input_context_ptr(&self) -> *const InputContext {
                (((self.input_ctx_ptr_hi() as u64) << 32) | ((self.input_ctx_ptr_lo() << 4) as u64))
                    as usize as *const InputContext
            }
            pub(super) fn set_input_context_ptr(&mut self, ptr: *const InputContext) {
                let ptr = ptr as usize as u64;
                debug_assert!(ptr & 0xF == 0);
                self.set_input_ctx_ptr_lo(((ptr & 0x00000000FFFFFFFF) >> 4) as u32);
                self.set_input_ctx_ptr_hi(((ptr & 0xFFFFFFFF00000000) >> 32) as u32);
            }
        }
        impl Default for EvaluateContextCommand {
            fn default() -> Self {
                let mut trb = Self { data: [0; 4] };
                trb.set_trb_type(Self::TYPE);
                trb
            }
        }
        impl Trb for EvaluateContextCommand {
            const TYPE: u8 = TypeId::EvaluateContextCommand as u8;
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

            getter!(data[3]: u32; 0x001F0000;  u8, endpoint_id_u8);
            getter!(data[3]: u32; 0xFF000000;  u8, pub slot_id);

            pub fn trb_pointer(&self) -> *const GenericTrb {
                (((self.trb_pointer_hi() as usize) << 32) | (self.trb_pointer_lo() as usize))
                    as *const GenericTrb
            }

            pub fn endpoint_id(&self) -> EndpointId {
                EndpointId::from_addr(self.endpoint_id_u8())
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

    use super::EndpointType;

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

        getter!(data[4]: u32; 0x0000FFFF; u16, pub average_trb_length);
        setter!(data[4]: u32; 0x0000FFFF; u16, pub set_average_trb_length);

        pub fn set_transfer_ring_buffer(&mut self, ptr: usize) {
            self.set_dequeue_pointer_lo((((ptr as u64) & 0x00000000FFFFFFFF) >> 4) as u32);
            self.set_dequeue_pointer_hi((((ptr as u64) & 0xFFFFFFFF00000000) >> 32) as u32);
        }
    }

    #[repr(transparent)]
    #[derive(Clone, Copy)]
    struct DeviceContextIndex(usize);
    impl From<EndpointId> for DeviceContextIndex {
        fn from(ep_id: EndpointId) -> Self {
            Self(ep_id.address() as usize)
        }
    }

    #[repr(C, align(64))]
    struct DeviceContext {
        slot_ctx: SlotContext,
        ep_ctxs: [EndpointContext; 31],
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
            self.input_control_ctx.add_context_flags |= 1;
            &mut self.slot_ctx
        }
        pub fn update_endpoint(&mut self, dci: DeviceContextIndex) -> &mut EndpointContext {
            self.input_control_ctx.add_context_flags |= 1 << dci.0;
            &mut self.ep_ctxs[dci.0 - 1]
        }
    }

    use super::{
        class_driver, descriptor, request_type, EndpointConfig, EndpointId, Request, SetupData,
    };
    use descriptor::{
        ConfigurationDescriptor, Descriptor, DeviceDescriptor, EndpointDescriptor, HidDescriptor,
        InterfaceDescriptor,
    };

    pub struct Device {
        ctx: *const DeviceContext,
        input_ctx: InputContext,
        doorbell: *mut DoorbellRegister,
        transfer_rings: [Option<TransferRing>; 31],
        command_trb: Option<GenericTrb>,
        slot_id: u8,
        speed: PortSpeed,

        buf: Buffer,
        init_phase: i8,
        num_configurations: u8,
        config_index: u8,
        ep_configs: FixedVec<EndpointConfig, 16>,
        class_drivers: FixedVec<&'static mut dyn class_driver::Driver, 16>,

        /// EP number --> class driver index
        class_driver_idxs: [Option<usize>; 16],

        /// Setup Data --> class driver index
        event_waiters: ArrayMap<SetupData, usize, 4>,

        /// {DataStage,StatusStage} TRB --> SetupData
        setup_data_map: ArrayMap<*const trb::GenericTrb, SetupData, 16>,
    }

    impl Device {
        const BUF_SIZE: usize = 1024;

        unsafe fn initialize_ptr(
            ptr: *mut Self,
            ctx: *const DeviceContext,
            doorbell: *mut DoorbellRegister,
            slot_id: u8,
            port: &Port,
        ) -> Result<*const InputContext> {
            {
                let ctx_ptr = addr_of_mut!((*ptr).ctx);
                ctx_ptr.write(ctx);

                let input_ctx_ptr = addr_of_mut!((*ptr).input_ctx);
                InputContext::initialize_ptr(input_ctx_ptr);

                let doorbell_ptr = addr_of_mut!((*ptr).doorbell);
                doorbell_ptr.write(doorbell);

                let transfer_rings_ptr =
                    addr_of_mut!((*ptr).transfer_rings) as *mut Option<TransferRing>;
                for i in 0..31 {
                    transfer_rings_ptr.add(i).write(None);
                }

                let command_trb_ptr = addr_of_mut!((*ptr).command_trb);
                command_trb_ptr.write(None);

                let slot_id_ptr = addr_of_mut!((*ptr).slot_id);
                slot_id_ptr.write(slot_id);

                let speed_ptr = addr_of_mut!((*ptr).speed);
                speed_ptr.write(port.speed());

                let init_phase_ptr = addr_of_mut!((*ptr).init_phase);
                init_phase_ptr.write(-1);

                let buf_ptr: *mut Buffer = addr_of_mut!((*ptr).buf);
                buf_ptr.write(Buffer::new(&mut *MALLOC.lock(), Self::BUF_SIZE, 64));

                let num_configurations_ptr = addr_of_mut!((*ptr).num_configurations);
                num_configurations_ptr.write(0);

                let config_index_ptr = addr_of_mut!((*ptr).config_index);
                config_index_ptr.write(0);

                let ep_configs_ptr = addr_of_mut!((*ptr).ep_configs);
                FixedVec::initialize_ptr(ep_configs_ptr);

                let class_drivers_ptr = addr_of_mut!((*ptr).class_drivers);
                FixedVec::initialize_ptr(class_drivers_ptr);

                let class_driver_idxs_ptr =
                    addr_of_mut!((*ptr).class_driver_idxs) as *mut Option<usize>;
                for i in 0..16 {
                    class_driver_idxs_ptr.add(i).write(None);
                }

                let event_waiters_ptr = addr_of_mut!((*ptr).event_waiters);
                ArrayMap::initialize_ptr(event_waiters_ptr);

                let setup_data_map_ptr = addr_of_mut!((*ptr).setup_data_map);
                ArrayMap::initialize_ptr(setup_data_map_ptr);
            }
            let device = &mut *ptr;

            let slot_ctx = device.input_ctx.enable_slot_context();
            slot_ctx.set_route_string(0);
            slot_ctx.set_root_hub_port_number(port.number());
            slot_ctx.set_context_entries(1);
            slot_ctx.set_speed(port.speed() as u8);

            let ep0_dci = EndpointId::DEFAULT_CONTROL_PIPE.into();
            let tr_buf = device.alloc_transfer_ring(ep0_dci, 32)?.buffer_ptr();
            let max_packet_size = port.speed().determine_max_packet_size_for_control_pipe();
            debug!(
                "port.speed() = {:?}, max_packet_size = {}",
                port.speed(),
                max_packet_size
            );

            let ep0_ctx = device.input_ctx.update_endpoint(ep0_dci);
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
            unsafe { (*self.ctx).slot_ctx.root_hub_port_number() }
        }

        pub fn is_initialized(&self) -> bool {
            self.init_phase == 4
        }

        fn initialize_phase0(&mut self, transfered_size: usize) -> Result<()> {
            debug_assert_eq!(transfered_size, 8);
            trace!("initialize_phase0 on slot_id={}", self.slot_id);

            debug_assert_eq!(self.buf[..].len(), Self::BUF_SIZE);
            let device_desc = descriptor::from_bytes::<DeviceDescriptor>(&self.buf[..8]).unwrap();
            debug!("USB release = {:04x}", device_desc.usb_release as u16);
            let ep0_ctx = self
                .input_ctx
                .update_endpoint(EndpointId::DEFAULT_CONTROL_PIPE.into());
            let max_packet_size = if device_desc.usb_release >= 0x0300 {
                (1 << device_desc.max_packet_size) as u16
            } else {
                device_desc.max_packet_size as u16
            };
            debug!(
                "update max_packet_size: {} => {}",
                ep0_ctx.max_packet_size(),
                max_packet_size
            );
            ep0_ctx.set_max_packet_size(max_packet_size);
            let mut eval_ctx_cmd = trb::EvaluateContextCommand::default();
            eval_ctx_cmd.set_input_context_ptr(&self.input_ctx);
            eval_ctx_cmd.set_slot_id(self.slot_id);
            self.command_trb = Some(eval_ctx_cmd.upcast().clone());

            Ok(())
        }

        fn initialize_phase1(&mut self, transfered_size: usize) -> Result<()> {
            trace!("initialize_phase1 on slot_id={}", self.slot_id);
            debug_assert_eq!(transfered_size, 18);
            let device_desc = descriptor::from_bytes::<DeviceDescriptor>(&self.buf[..18]).unwrap();

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
                ConfigurationDescriptor::TYPE,
                self.config_index,
                Self::BUF_SIZE,
            )
        }

        fn initialize_phase2(&mut self, transfered_size: usize) -> Result<()> {
            trace!("initialize_phase2 on slot_id={}", self.slot_id);

            // TODO: handle the case where total_length > BUF_SIZE
            assert_eq!(
                descriptor::from_bytes::<ConfigurationDescriptor>(&self.buf[..])
                    .unwrap()
                    .total_length as usize,
                transfered_size
            );

            let mut desc_itr = descriptor::DescIter::new(&self.buf[..transfered_size]);
            while let Some(if_desc) = desc_itr.next::<InterfaceDescriptor>() {
                let class_driver = match Self::new_class_driver(if_desc) {
                    Ok(driver) => driver,
                    Err(Error::UnsupportedInterface) => continue,
                    Err(e) => return Err(e),
                };

                let class_driver_idx = self.class_drivers.push(class_driver).unwrap();

                let mut num_endpoints = 0;
                debug!("if_desc.num_endpoints = {}", if_desc.num_endpoints);
                while num_endpoints < if_desc.num_endpoints {
                    if let Some(ep_desc) = desc_itr.next::<EndpointDescriptor>() {
                        let conf = EndpointConfig::from(ep_desc);
                        debug!("{:?}", conf);
                        let ep_id = conf.ep_id;
                        self.ep_configs.push(conf);
                        num_endpoints += 1;
                        self.class_driver_idxs[ep_id.number() as usize] = Some(class_driver_idx);
                    } else if let Some(hid_desc) = desc_itr.next::<HidDescriptor>() {
                        debug!("{:?}", hid_desc);
                    }
                }
            }

            if self.class_drivers.len() == 0 {
                warn!("No available class drivers found");
                Ok(())
            } else {
                self.init_phase = 3;

                let conf_desc =
                    descriptor::from_bytes::<ConfigurationDescriptor>(&self.buf[..]).unwrap();
                debug!(
                    "issuing Set Configuration: conf_val = {}",
                    conf_desc.configuration_value
                );
                let conf_val = conf_desc.configuration_value;
                self.set_configuration(EndpointId::DEFAULT_CONTROL_PIPE, conf_val)
            }
        }

        fn initialize_phase3(&mut self) -> Result<()> {
            trace!("initialize_phase3 on slot_id={}", self.slot_id);
            for conf in self.ep_configs.iter() {
                let driver_idx = self.class_driver_idxs[conf.ep_id.number() as usize];
                self.class_drivers
                    .get_mut(driver_idx.unwrap())
                    .unwrap()
                    .set_endpoint(conf)?;
            }
            self.init_phase = 4;
            debug!("slot_id = {}: initialized", self.slot_id);
            Ok(())
        }

        fn new_class_driver(
            if_desc: &InterfaceDescriptor,
        ) -> Result<&'static mut dyn class_driver::Driver> {
            let class = if_desc.interface_class;
            let sub = if_desc.interface_sub_class;
            let proto = if_desc.interface_protocol;
            match (class, sub, proto) {
                (3, 1, 1) => {
                    info!("keyboard found");
                    use class_driver::HidKeyboardDriver;
                    let keyboard_driver = unsafe {
                        let keyboard_driver: &mut MaybeUninit<HidKeyboardDriver> = MALLOC
                            .lock()
                            .alloc_obj::<HidKeyboardDriver>()
                            .ok_or(Error::NoEnoughMemory)?
                            .as_mut();
                        keyboard_driver
                            .as_mut_ptr()
                            .write(HidKeyboardDriver::new(if_desc.interface_number)?);
                        core::mem::transmute::<
                            &mut MaybeUninit<HidKeyboardDriver>,
                            &mut HidKeyboardDriver,
                        >(keyboard_driver)
                    };
                    Ok(keyboard_driver)
                }
                (3, 1, 2) => {
                    info!("mouse found");
                    use class_driver::HidMouseDriver;
                    let mouse_driver = unsafe {
                        let mouse_driver: &mut MaybeUninit<HidMouseDriver> = MALLOC
                            .lock()
                            .alloc_obj::<HidMouseDriver>()
                            .ok_or(Error::NoEnoughMemory)?
                            .as_mut();
                        mouse_driver
                            .as_mut_ptr()
                            .write(HidMouseDriver::new(if_desc.interface_number)?);
                        core::mem::transmute::<&mut MaybeUninit<HidMouseDriver>, &mut HidMouseDriver>(
                            mouse_driver,
                        )
                    };
                    Ok(mouse_driver)
                }
                _ => Err(Error::UnsupportedInterface),
            }
        }

        fn configure_endpoints(&mut self) -> Result<*const InputContext> {
            {
                let input_ctrl_ctx_ptr: *mut u8 =
                    &mut self.input_ctx.input_control_ctx as *mut _ as *mut u8;
                unsafe { input_ctrl_ctx_ptr.write_bytes(0, size_of::<InputControlContext>()) };

                let src = unsafe { &(*self.ctx).slot_ctx as *const SlotContext };
                let dst = &mut self.input_ctx.slot_ctx as *mut SlotContext;
                unsafe { core::ptr::copy(src, dst, 1) };
            }

            let slot_ctx = self.input_ctx.enable_slot_context();
            slot_ctx.set_context_entries(31);

            for i in 0..self.ep_configs.len() {
                let ep_dci = DeviceContextIndex::from(self.ep_configs.get(i).unwrap().ep_id);
                let tr_buf_ptr = self.alloc_transfer_ring(ep_dci, 32)?.buffer_ptr();

                let config = self.ep_configs.get(i).unwrap();
                let ep_ctx = self.input_ctx.update_endpoint(ep_dci);

                match config.ep_type {
                    EndpointType::Control => {
                        ep_ctx.set_endpoint_type(4);
                    }
                    EndpointType::Isochronous => {
                        ep_ctx.set_endpoint_type(if config.ep_id.is_in() { 5 } else { 1 });
                    }
                    EndpointType::Bulk => {
                        ep_ctx.set_endpoint_type(if config.ep_id.is_in() { 6 } else { 2 });
                    }
                    EndpointType::Interrupt => {
                        ep_ctx.set_endpoint_type(if config.ep_id.is_in() { 7 } else { 3 });
                    }
                }

                // [0..255] --> [0..7]
                fn interval_map(v: u8) -> u8 {
                    if v == 0 {
                        0
                    } else {
                        7 - v.leading_zeros() as u8
                    }
                }
                let interval = match self.speed {
                    PortSpeed::Full | PortSpeed::Low => {
                        if config.ep_type == EndpointType::Isochronous {
                            config.interval + 2
                        } else {
                            interval_map(config.interval) + 3
                        }
                    }
                    _ => config.interval - 1,
                };
                ep_ctx.set_interval(interval);

                ep_ctx.set_max_packet_size(config.max_packet_size);
                ep_ctx.set_average_trb_length(1);
                ep_ctx.set_transfer_ring_buffer(tr_buf_ptr as usize);
                ep_ctx.set_dequeue_cycle_state(1);
                ep_ctx.set_max_primary_streams(0);
                ep_ctx.set_mult(0);
                ep_ctx.set_error_count(3);
            }

            Ok(&self.input_ctx as *const InputContext)
        }

        fn on_endpoints_configured(&mut self) -> Result<()> {
            for idx in 0..self.class_drivers.len() {
                let class_driver = self.class_drivers.get_mut(idx).expect("index out of range");
                match class_driver.on_endpoints_configured()? {
                    class_driver::TransferRequest::NoOp => continue,
                    class_driver::TransferRequest::ControlOut(setup_data) => {
                        self.control_out(
                            EndpointId::DEFAULT_CONTROL_PIPE,
                            setup_data,
                            Some(idx),
                            None,
                            0,
                        )?;
                    }
                    _ => unimplemented!(),
                }
            }
            Ok(())
        }

        fn on_command_completion_event_received(&mut self, issuer_type: u8) -> Result<()> {
            match issuer_type {
                trb::AddressDeviceCommand::TYPE => {
                    self.init_phase = 0;
                    self.get_descriptor(
                        EndpointId::DEFAULT_CONTROL_PIPE,
                        DeviceDescriptor::TYPE,
                        0,
                        8, // request first 8-bytes
                    )
                }
                trb::EvaluateContextCommand::TYPE => {
                    self.init_phase = 1;
                    self.get_descriptor(
                        EndpointId::DEFAULT_CONTROL_PIPE,
                        DeviceDescriptor::TYPE,
                        0,
                        18, // entire descriptor
                    )
                }
                _ => {
                    unreachable!();
                }
            }
        }

        fn on_transfer_event_received(&mut self, trb: &trb::TransferEvent) -> Result<()> {
            if !(trb.completion_code() == 1 /* Success */ || trb.completion_code() == 13/* Short Packet*/)
            {
                warn!(
                    "on_transfer_event_received: invalid trb = {:?} (completion code: {})",
                    trb.upcast(),
                    trb.completion_code(),
                );
                return Err(Error::TransferFailed {
                    slot_id: trb.slot_id(),
                });
            }

            trace!(
                "device::on_transfer_event_received: issuer TRB = {:p}",
                trb.trb_pointer()
            );

            let issuer_trb = unsafe { &*trb.trb_pointer() };
            let residual_bytes = trb.trb_transfer_length() as usize;

            if let Some(normal) = issuer_trb.downcast_ref::<trb::Normal>() {
                let transfer_length = normal.trb_transfer_length() as usize - residual_bytes;
                return self.on_interrupt_completed(
                    trb.endpoint_id(),
                    NonNull::new(normal.data_buffer()).expect("data buffer null"),
                    transfer_length,
                );
            }

            let setup_data = match self
                .setup_data_map
                .remove(&(issuer_trb as *const trb::GenericTrb))
            {
                None => {
                    warn!("No Correspoinding Setup Stage TRB");
                    return Err(Error::NoCorrespondingSetupStage);
                }
                Some((_, setup_data)) => setup_data,
            };

            let (buf, transfered_size) =
                if let Some(data_stage) = issuer_trb.downcast_ref::<trb::DataStage>() {
                    let buf = NonNull::new(data_stage.data_buffer()).expect("null pointer");
                    trace!("residual_bytes = {}", residual_bytes);
                    (Some(buf), setup_data.length as usize - residual_bytes)
                } else if let Some(_status_stage) = issuer_trb.downcast_ref::<trb::StatusStage>() {
                    (None, 0)
                } else {
                    unimplemented!();
                };

            self.on_control_completed(trb.endpoint_id(), setup_data, buf, transfered_size)
        }

        fn on_control_completed(
            &mut self,
            ep_id: EndpointId,
            setup_data: SetupData,
            buf_ptr: Option<NonNull<u8>>,
            transfered_size: usize,
        ) -> Result<()> {
            debug!(
                "device::on_control_completed: transfered_size = {}, dir = {}",
                transfered_size,
                setup_data.direction(),
            );
            match self.init_phase {
                0 => {
                    unsafe { self.buf.attach(buf_ptr.unwrap()) };
                    let buf = &self.buf[..transfered_size];
                    if setup_data.request == Request::GetDescriptor as u8
                        && descriptor::from_bytes::<DeviceDescriptor>(buf).is_some()
                    {
                        self.initialize_phase0(transfered_size)
                    } else {
                        warn!("failed to go initialize_phase0");
                        Err(Error::InvalidPhase)
                    }
                }
                1 => {
                    unsafe { self.buf.attach(buf_ptr.unwrap()) };
                    let buf = &self.buf[..transfered_size];
                    if setup_data.request == Request::GetDescriptor as u8
                        && descriptor::from_bytes::<DeviceDescriptor>(buf).is_some()
                    {
                        self.initialize_phase1(transfered_size)
                    } else {
                        warn!("failed to go initialize_phase1");
                        Err(Error::InvalidPhase)
                    }
                }
                2 => {
                    unsafe { self.buf.attach(buf_ptr.unwrap()) };
                    let buf = &self.buf[..transfered_size];
                    if setup_data.request == Request::GetDescriptor as u8
                        && descriptor::from_bytes::<ConfigurationDescriptor>(buf).is_some()
                    {
                        self.initialize_phase2(transfered_size)
                    } else {
                        warn!("failed to go initialize_phase2");
                        Err(Error::InvalidPhase)
                    }
                }
                3 => {
                    if setup_data.request == Request::SetConfiguration as u8 {
                        debug_assert!(buf_ptr.is_none() && self.buf.own());
                        self.initialize_phase3()
                    } else {
                        warn!("failed to go initialize_phase3");
                        Err(Error::InvalidPhase)
                    }
                }
                _ => match self.event_waiters.get(&setup_data) {
                    Some(&w_idx) => {
                        let w = self
                            .class_drivers
                            .get_mut(w_idx)
                            .expect("uninitialized class driver");
                        match w.on_control_completed(ep_id, setup_data, buf_ptr, transfered_size)? {
                            class_driver::TransferRequest::InterruptIn {
                                ep_id,
                                buf_ptr,
                                size,
                            } => self.interrupt_in(ep_id, buf_ptr, size),
                            _ => unimplemented!(),
                        }
                    }
                    None => Err(Error::NoWaiter),
                },
            }
        }

        fn on_interrupt_completed(
            &mut self,
            ep_id: EndpointId,
            buf_ptr: NonNull<u8>,
            transfered_size: usize,
        ) -> Result<()> {
            trace!(
                "Device::on_interrupt_completed: EP addr = {}",
                ep_id.address()
            );
            if let Some(driver_idx) = self.class_driver_idxs[ep_id.number() as usize] {
                let w = self.class_drivers.get_mut(driver_idx).unwrap();
                match w.on_interrupt_completed(ep_id, buf_ptr, transfered_size)? {
                    class_driver::TransferRequest::InterruptIn {
                        ep_id,
                        buf_ptr,
                        size,
                    } => {
                        self.interrupt_in(ep_id, buf_ptr, size)?;
                    }
                    _ => unimplemented!(),
                }
            }
            Ok(())
        }

        fn ring_doorbell(&mut self, dci: DeviceContextIndex) {
            trace!("ring the doorbell with target {}", dci.0);
            unsafe { (*self.doorbell).ring(dci.0 as u8) };
        }

        fn control_in(
            &mut self,
            ep_id: EndpointId,
            setup_data: SetupData,
            issuer_idx: Option<usize>,
            buf_ptr: Option<NonNull<u8>>,
            size: usize,
        ) -> Result<()> {
            if let Some(issuer_idx) = issuer_idx {
                self.event_waiters
                    .insert(setup_data.clone(), issuer_idx)
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

            if let Some(buf_ptr) = buf_ptr {
                let setup_stage = trb::SetupStage::new_in_data_stage(setup_data.clone());
                tr.push(setup_stage.upcast());

                let mut data_stage = trb::DataStage::new_in(buf_ptr.as_ptr(), size);
                data_stage.set_interrupt_on_completion(1);
                let data_stage_trb_ptr = tr.push(data_stage.upcast()) as *const trb::GenericTrb;

                let mut status_stage = trb::StatusStage::default();
                status_stage.set_direction(0);

                let status_stage_trb_ptr = tr.push(status_stage.upcast());
                debug!("status_stage_trb = {:p}", status_stage_trb_ptr);

                self.setup_data_map
                    .insert(data_stage_trb_ptr, setup_data)
                    .map_err(|e| match e {
                        ArrayMapError::NoSpace => Error::TooManyWaiters,
                        ArrayMapError::SameKeyRegistered => {
                            panic!("same data_stage_trb_ptr registered")
                        }
                    })?;

                self.ring_doorbell(dci);
            } else {
                unimplemented!();
            }

            Ok(())
        }

        fn control_out(
            &mut self,
            ep_id: EndpointId,
            setup_data: SetupData,
            issuer_idx: Option<usize>,
            buf_ptr: Option<NonNull<u8>>,
            size: usize,
        ) -> Result<()> {
            if let Some(issuer_idx) = issuer_idx {
                self.event_waiters
                    .insert(setup_data.clone(), issuer_idx)
                    .map_err(|e| match e {
                        ArrayMapError::NoSpace => Error::TooManyWaiters,
                        ArrayMapError::SameKeyRegistered => {
                            panic!("same setup_data registered")
                        }
                    })?;
            }

            debug!("device::control_out: ep addr = {}", ep_id.address());
            if 15 < ep_id.number() {
                return Err(Error::InvalidEndpointNumber);
            }

            let dci = DeviceContextIndex::from(ep_id);
            let tr = self.transfer_rings[dci.0 - 1]
                .as_mut()
                .ok_or(Error::TransferRingNotSet)?;

            if let Some(_buf_ptr) = buf_ptr {
                let _size = size;
                unimplemented!();
            } else {
                let setup_stage = trb::SetupStage::new_no_data_stage(setup_data.clone());
                tr.push(setup_stage.upcast());

                let mut status_stage = trb::StatusStage::default();
                status_stage.set_direction(1);
                status_stage.set_interrupt_on_completion(1);
                let status_stage_trb_ptr = tr.push(status_stage.upcast());
                debug!("status_stage_trb = {:p}", status_stage_trb_ptr);

                self.setup_data_map
                    .insert(status_stage_trb_ptr, setup_data)
                    .map_err(|e| match e {
                        ArrayMapError::NoSpace => Error::TooManyWaiters,
                        ArrayMapError::SameKeyRegistered => {
                            panic!("same data_stage_trb_ptr registered")
                        }
                    })?;

                self.ring_doorbell(dci);
            }
            Ok(())
        }

        fn interrupt_in(
            &mut self,
            ep_id: EndpointId,
            buf_ptr: Option<NonNull<u8>>,
            size: usize,
        ) -> Result<()> {
            let dci = DeviceContextIndex::from(ep_id);
            let tr = self.transfer_rings[dci.0 - 1]
                .as_mut()
                .ok_or(Error::TransferRingNotSet)?;

            let mut normal = trb::Normal::default();
            normal.set_data_buffer(buf_ptr.map(|ptr| ptr.as_ptr()).unwrap_or(null_mut()));
            normal.set_trb_transfer_length(size as u32);
            normal.set_interrupt_on_short_packet(1);
            normal.set_interrupt_on_completion(1);

            tr.push(normal.upcast());
            self.ring_doorbell(dci);

            Ok(())
        }

        fn get_descriptor(
            &mut self,
            ep_id: EndpointId,
            desc_type: u8,
            desc_index: u8,
            transfer_size: usize,
        ) -> Result<()> {
            let mut setup_data = SetupData::default();
            setup_data.set_direction(request_type::Direction::DeviceToHost as u8);
            setup_data.set_typ(request_type::Type::Standard as u8);
            setup_data.set_recipient(request_type::Recipient::Device as u8);
            setup_data.request = Request::GetDescriptor as u8;
            setup_data.value = ((desc_type as u16) << 8) | (desc_index as u16);
            setup_data.index = 0;

            let buf = self.buf.detach();
            setup_data.length = transfer_size as u16;
            self.control_in(ep_id, setup_data, None, Some(buf), transfer_size)
        }

        fn set_configuration(&mut self, ep_id: EndpointId, config_value: u8) -> Result<()> {
            let mut setup_data = SetupData::default();
            setup_data.set_direction(request_type::Direction::HostToDevice as u8);
            setup_data.set_typ(request_type::Type::Standard as u8);
            setup_data.set_recipient(request_type::Recipient::Device as u8);
            setup_data.request = Request::SetConfiguration as u8;
            setup_data.value = config_value as u16;
            setup_data.index = 0;
            setup_data.length = 0;
            self.control_out(ep_id, setup_data, None, None, 0)
        }
    }

    struct DeviceManager {
        devices: &'static mut [Option<&'static mut Device>],
        device_context_pointers: *mut [*const DeviceContext],
        doorbells: *mut DoorbellRegister, // 1-255
    }
    impl DeviceManager {
        pub fn new(
            max_slots: usize,
            doorbells: *mut DoorbellRegister,
            scratchpad_buf_arr: *const *const u8,
        ) -> Result<Self> {
            let mut malloc = MALLOC.lock();

            let devices: &mut [MaybeUninit<Option<&'static mut Device>>] = unsafe {
                malloc
                    .alloc_slice::<Option<&'static mut Device>>(max_slots + 1)
                    .ok_or(Error::NoEnoughMemory)?
                    .as_mut()
            };
            for p in devices.iter_mut() {
                *p = MaybeUninit::new(None);
            }
            let devices = unsafe {
                core::mem::transmute::<
                    &mut [MaybeUninit<Option<&'static mut Device>>],
                    &mut [Option<&'static mut Device>],
                >(devices)
            };

            // NOTE: DCBAA: alignment = 64-bytes, boundary = PAGESIZE
            let dcbaap: &mut [MaybeUninit<*const DeviceContext>] = unsafe {
                malloc
                    .alloc_slice_ext::<*const DeviceContext>(max_slots + 1, 64, None)
                    .ok_or(Error::NoEnoughMemory)?
                    .as_mut()
            };
            for p in dcbaap.iter_mut() {
                *p = MaybeUninit::new(null());
            }
            dcbaap[0] = MaybeUninit::new(scratchpad_buf_arr as *const _);

            let dcbaap = unsafe {
                core::mem::transmute::<
                    &mut [MaybeUninit<*const DeviceContext>],
                    &mut [*const DeviceContext],
                >(dcbaap)
            };
            let dcbaap = dcbaap as *mut [*const DeviceContext];

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
            if self.devices[slot_id].is_some() {
                return Err(Error::DeviceAlreadyAllocated);
            }

            let device_ctx: &mut MaybeUninit<DeviceContext> = unsafe {
                MALLOC
                    .lock()
                    .alloc_obj::<DeviceContext>()
                    .ok_or(Error::NoEnoughMemory)?
                    .as_mut()
            };
            unsafe { DeviceContext::initialize_ptr(device_ctx.as_mut_ptr()) };
            let device_ctx =
                device_ctx as *const MaybeUninit<DeviceContext> as *const DeviceContext;

            let device: &mut MaybeUninit<Device> = unsafe {
                MALLOC
                    .lock()
                    .alloc_obj::<Device>()
                    .ok_or(Error::NoEnoughMemory)?
                    .as_mut()
            };

            let input_ctx = unsafe {
                Device::initialize_ptr(
                    device.as_mut_ptr() as *mut Device,
                    device_ctx,
                    self.doorbells.add(slot_id - 1),
                    slot_id as u8,
                    port,
                )?
            };

            let device =
                unsafe { core::mem::transmute::<&mut MaybeUninit<Device>, &mut Device>(device) };

            self.devices[slot_id] = Some(device);

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
            debug_assert!((ptr as usize) % 64 == 0);
            ptr
        }

        pub fn find_by_slot(&self, slot_id: u8) -> Option<&Device> {
            self.devices
                .get(slot_id as usize)
                .and_then(|dev| dev.as_deref())
        }

        pub fn find_by_slot_mut(&mut self, slot_id: u8) -> Option<&mut Device> {
            self.devices
                .get_mut(slot_id as usize)
                .and_then(|dev| dev.as_deref_mut())
        }
    }
    // TODO: impl Drop for DeviceManager

    #[derive(Debug)]
    #[repr(u8)]
    pub enum PortSpeed {
        Full = 1,
        Low = 2,
        High = 3,
        Super = 4,
        SuperSpeedPlus = 5,
    }
    impl PortSpeed {
        pub fn determine_max_packet_size_for_control_pipe(&self) -> u16 {
            match self {
                Self::SuperSpeedPlus | Self::Super => 512,
                Self::High => 64,
                Self::Full => {
                    warn!("Max Packet Size of FullSpeed devices is one of 8, 16, 32, or 64");
                    8
                }
                Self::Low => 8,
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
                5 => Self::SuperSpeedPlus,
                _ => panic!("unknown speed: {}", speed),
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
                    portsc.data &= 0b_0000_1110_1111_1110_1100_0011_1110_0000;
                    portsc.data |= 0b_0000_0000_0000_0010_0000_0000_0001_0000; // Write 1 to PR and CSC
                })
            };
            while unsafe { (*self.regs).portsc.read().port_reset() == 1 } {}
            trace!("port reset on port {}", self.port_num);
        }

        pub fn clear_connect_status_change(&mut self) {
            unsafe {
                (*self.regs).portsc.modify_with(|portsc| {
                    portsc.data &= 0b_0000_1110_1111_1110_1100_0011_1110_0000;
                    portsc.set_connect_status_change(1);
                })
            };
            trace!("clear CCS on port {}", self.port_num);
        }

        pub fn clear_port_reset_change(&mut self) {
            unsafe {
                (*self.regs).portsc.modify_with(|portsc| {
                    portsc.data &= 0b_0000_1110_1111_1110_1100_0011_1110_0000;
                    portsc.set_port_reset_change(1);
                })
            };
            trace!("clear PRC on port {}", self.port_num);
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

        pub fn bits(&self) -> u32 {
            unsafe { (*self.regs).portsc.read().data }
        }
    }
}

mod request_type {
    #![allow(dead_code)]

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
    pub enum Direction {
        HostToDevice = 0,
        DeviceToHost = 1,
    }
}

#[repr(u8)]
enum Request {
    GetDescriptor = 6,
    SetConfiguration = 9,
}
#[repr(u8)]
enum HidRequest {
    SetProtocol = 11,
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum EndpointType {
    Control = 0,
    Isochronous = 1,
    Bulk = 2,
    Interrupt = 3,
}
impl core::convert::TryFrom<u8> for EndpointType {
    type Error = Error;
    fn try_from(ty: u8) -> core::result::Result<Self, Self::Error> {
        match ty {
            0 => Ok(Self::Control),
            1 => Ok(Self::Isochronous),
            2 => Ok(Self::Bulk),
            3 => Ok(Self::Interrupt),
            _ => Err(Error::InvalidEndpointType { ty }),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct EndpointId {
    addr: u8,
}
impl EndpointId {
    const DEFAULT_CONTROL_PIPE: EndpointId = EndpointId { addr: 1 };

    pub fn from_addr(addr: u8) -> Self {
        Self { addr }
    }
    pub fn from_number_in(ep_num: u8) -> Self {
        Self {
            addr: (ep_num << 1) | 1,
        }
    }
    pub fn from_number_out(ep_num: u8) -> Self {
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
}

#[derive(Debug, Clone)]
pub struct EndpointConfig {
    ep_id: EndpointId,
    ep_type: EndpointType,
    max_packet_size: u16,
    interval: u8,
}
impl From<&descriptor::EndpointDescriptor> for EndpointConfig {
    fn from(ep_desc: &descriptor::EndpointDescriptor) -> Self {
        Self {
            ep_id: EndpointId::from_number_in(ep_desc.number()),
            ep_type: <EndpointType as core::convert::TryFrom<u8>>::try_from(
                ep_desc.transfer_type(),
            )
            .expect("invalid EndpointType"),
            max_packet_size: ep_desc.max_packet_size,
            interval: ep_desc.interval,
        }
    }
}

mod descriptor {
    #[repr(u8)]
    pub enum Type {
        Device = 1,
        Configuration = 2,
        Interface = 4,
        Endpoint = 5,
        Hid = 33,
    }

    pub trait Descriptor {
        const TYPE: u8;
    }

    pub fn from_bytes<D: Descriptor>(bytes: &[u8]) -> Option<&D> {
        if !bytes.is_empty() && bytes[0] == core::mem::size_of::<D>() as u8 && bytes[1] == D::TYPE {
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

    pub struct DescIter<'buf> {
        buf: &'buf [u8],
    }
    impl<'buf> DescIter<'buf> {
        pub fn new(conf_desc: &'buf [u8]) -> Self {
            Self { buf: conf_desc }
        }
        pub fn next<D: Descriptor>(&mut self) -> Option<&'buf D> {
            loop {
                let sz = self.buf[0] as usize;
                if self.buf.len() < sz {
                    return None;
                }
                self.buf = &self.buf[sz..];
                if self.buf.is_empty() {
                    return None;
                }
                if let Some(desc) = from_bytes(self.buf) {
                    return Some(desc);
                }
            }
        }
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

    #[repr(C, packed)]
    pub struct EndpointDescriptor {
        pub length: u8,
        pub descriptor_type: u8,
        pub endpoint_address: u8,
        pub attributes: u8,
        pub max_packet_size: u16,
        pub interval: u8,
    }
    impl EndpointDescriptor {
        getter!(endpoint_address: u8; 0b00001111; u8, pub number);
        getter!(endpoint_address: u8; 0b10000000; u8, pub dir_in);
        getter!(attributes: u8; 0b00000011; u8, pub transfer_type);
        getter!(attributes: u8; 0b00001100; u8, pub sync_type);
        getter!(attributes: u8; 0b00110000; u8, pub usage_type);
    }
    impl Descriptor for EndpointDescriptor {
        const TYPE: u8 = Type::Endpoint as u8;
    }

    #[repr(C, packed)]
    pub struct HidDescriptor {
        pub length: u8,
        pub descriptor_type: u8,
        pub hid_release: u16,
        pub country_code: u8,
        pub num_descriptors: u8,
    }
    impl Descriptor for HidDescriptor {
        const TYPE: u8 = Type::Hid as u8;
    }
    impl core::fmt::Debug for HidDescriptor {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            let hid_release = self.hid_release;
            write!(f, "HidDescriptor {{ length: {}, descriptor_type: {}, hid_release: {}, country_code: {}, num_descriptors: {} }}", self.length, self.descriptor_type, hid_release, self.country_code, self.num_descriptors)
        }
    }
}

pub mod class_driver {
    use super::Result;
    use super::{
        request_type, EndpointConfig, EndpointId, EndpointType, HidRequest, SetupData, MALLOC,
    };
    use crate::utils::Buffer;

    use core::ptr::NonNull;

    pub enum TransferRequest {
        NoOp,
        ControlOut(SetupData),
        InterruptIn {
            ep_id: EndpointId,
            buf_ptr: Option<NonNull<u8>>,
            size: usize,
        },
    }

    pub trait Driver {
        fn set_endpoint(&mut self, config: &EndpointConfig) -> Result<()>;
        fn on_endpoints_configured(&mut self) -> Result<TransferRequest>;
        fn on_control_completed(
            &mut self,
            ep_id: EndpointId,
            setup_data: SetupData,
            buf_ptr: Option<NonNull<u8>>,
            transfered_size: usize,
        ) -> Result<TransferRequest>;
        fn on_interrupt_completed(
            &mut self,
            ep_id: EndpointId,
            buf_ptr: NonNull<u8>,
            transfered_size: usize,
        ) -> Result<TransferRequest>;
    }

    pub struct HidDriver {
        interface_idx: u8,
        in_packet_size: usize,
        ep_interrupt_in: Option<EndpointId>,
        ep_interrupt_out: Option<EndpointId>,
        init_phase: u8,
        buf: Buffer,
        prev_buf: Buffer,
    }

    impl HidDriver {
        const BUF_SIZE: usize = 1024;

        pub fn new(interface_idx: u8, in_packet_size: usize) -> Result<Self> {
            let mut malloc = MALLOC.lock();
            Ok(Self {
                interface_idx,
                in_packet_size,
                ep_interrupt_in: None,
                ep_interrupt_out: None,
                buf: Buffer::new(&mut *malloc, Self::BUF_SIZE, 64),
                prev_buf: Buffer::new(&mut *malloc, Self::BUF_SIZE, 64),
                init_phase: 0,
            })
        }

        fn swap_buffer(&mut self) {
            let buf = &mut self.buf;
            let prev_buf = &mut self.prev_buf;
            core::mem::swap(buf, prev_buf);
        }

        pub fn buffer(&mut self) -> &mut [u8] {
            &mut self.prev_buf[..]
        }
    }

    impl Driver for HidDriver {
        fn set_endpoint(&mut self, config: &EndpointConfig) -> Result<()> {
            if config.ep_type == EndpointType::Interrupt {
                if config.ep_id.is_in() {
                    self.ep_interrupt_in = Some(config.ep_id);
                } else {
                    self.ep_interrupt_out = Some(config.ep_id);
                }
            }
            Ok(())
        }

        fn on_control_completed(
            &mut self,
            _ep_id: EndpointId,
            _setup_data: SetupData,
            buf_ptr: Option<NonNull<u8>>,
            transfered_size: usize,
        ) -> Result<TransferRequest> {
            trace!(
                "HidDriver::on_control_completed: phase = {}, transfered_size = {}",
                self.init_phase,
                transfered_size
            );

            match self.init_phase {
                1 => {
                    debug_assert!(buf_ptr.is_none());
                    self.init_phase = 2;
                    let buf_ptr = self.buf.detach();
                    Ok(TransferRequest::InterruptIn {
                        ep_id: self.ep_interrupt_in.expect("Endpoint not initialized"),
                        buf_ptr: Some(buf_ptr),
                        size: self.in_packet_size,
                    })
                }
                _ => unimplemented!(),
            }
        }

        fn on_interrupt_completed(
            &mut self,
            ep_id: EndpointId,
            buf_ptr: NonNull<u8>,
            transfered_size: usize,
        ) -> Result<TransferRequest> {
            if ep_id.is_in() {
                debug_assert!(transfered_size <= self.in_packet_size);

                unsafe { self.buf.attach(buf_ptr) };
                self.swap_buffer();
                let buf_ptr = self.buf.detach();

                Ok(TransferRequest::InterruptIn {
                    ep_id: self.ep_interrupt_in.expect("Endpoint not initialized"),
                    buf_ptr: Some(buf_ptr),
                    size: self.in_packet_size,
                })
            } else {
                unreachable!();
            }
        }

        fn on_endpoints_configured(&mut self) -> Result<TransferRequest> {
            let mut setup_data = SetupData::default();
            setup_data.set_direction(request_type::Direction::HostToDevice as u8);
            setup_data.set_typ(request_type::Type::Class as u8);
            setup_data.set_recipient(request_type::Recipient::Interface as u8);
            setup_data.request = HidRequest::SetProtocol as u8;
            setup_data.value = 0; // boot protocol
            setup_data.index = self.interface_idx as u16;
            setup_data.length = 0;

            self.init_phase = 1;
            Ok(TransferRequest::ControlOut(setup_data))
        }
    }

    pub struct HidMouseDriver {
        hid_driver: HidDriver,
    }
    impl HidMouseDriver {
        pub fn new(interface_idx: u8) -> Result<Self> {
            Ok(Self {
                hid_driver: HidDriver::new(interface_idx, 8)?,
            })
        }
    }
    impl Driver for HidMouseDriver {
        fn set_endpoint(&mut self, config: &EndpointConfig) -> Result<()> {
            self.hid_driver.set_endpoint(config)
        }
        fn on_control_completed(
            &mut self,
            ep_id: EndpointId,
            setup_data: SetupData,
            buf_ptr: Option<NonNull<u8>>,
            transfered_size: usize,
        ) -> Result<TransferRequest> {
            self.hid_driver
                .on_control_completed(ep_id, setup_data, buf_ptr, transfered_size)
        }
        fn on_interrupt_completed(
            &mut self,
            ep_id: EndpointId,
            buf_ptr: NonNull<u8>,
            transfered_size: usize,
        ) -> Result<TransferRequest> {
            let req = self
                .hid_driver
                .on_interrupt_completed(ep_id, buf_ptr, transfered_size)?;

            // FIXME
            {
                use crate::sync::spin::SpinMutex;
                static MOUSE_CURSOR_POS: SpinMutex<(isize, isize)> =
                    SpinMutex::new("mouse_cursor", (0, 0));

                let button = self.hid_driver.buffer()[0];
                let dx = self.hid_driver.buffer()[1];
                let dy = self.hid_driver.buffer()[2];

                let dx = if dx >= 128 {
                    (dx as isize) - 256
                } else {
                    dx as isize
                };
                let dy = if dy >= 128 {
                    (dy as isize) - 256
                } else {
                    dy as isize
                };

                use crate::global::lock_frame_buffer;
                use crate::graphics::{Color, Render};

                let mut cursor = MOUSE_CURSOR_POS.lock();
                let (mut x, mut y) = *cursor;

                if button == 0 {
                    lock_frame_buffer(|frame_buffer| {
                        frame_buffer.draw_filled_rect(x, y, 32, 32, Color::BLACK)
                    });
                }

                x = x.wrapping_add(dx);
                y = y.wrapping_add(dy);
                debug!("mouse: button:{}, dx: {:3}, dy: {:3}", button, dx, dy);

                let color = if button & 0b01 != 0 {
                    Color { r: 0, g: 255, b: 0 }
                } else if button & 0b10 != 0 {
                    Color { r: 0, g: 0, b: 255 }
                } else {
                    Color { r: 255, g: 0, b: 0 }
                };

                lock_frame_buffer(|frame_buffer| {
                    frame_buffer.draw_filled_rect(x, y, 32, 32, color)
                });
                *cursor = (x, y);
            }

            Ok(req)
        }
        fn on_endpoints_configured(&mut self) -> Result<TransferRequest> {
            self.hid_driver.on_endpoints_configured()
        }
    }

    pub struct HidKeyboardDriver {
        hid_driver: HidDriver,
        prev: [u8; 6],
    }
    impl HidKeyboardDriver {
        pub fn new(interface_idx: u8) -> Result<Self> {
            Ok(Self {
                hid_driver: HidDriver::new(interface_idx, 8)?,
                prev: [0; 6],
            })
        }
        fn key2ascii(shift: bool, keycode: u8) -> Option<char> {
            match (shift, keycode) {
                (false, 0x04) => Some('a'),
                (false, 0x05) => Some('b'),
                (false, 0x06) => Some('c'),
                (false, 0x07) => Some('d'),
                (false, 0x08) => Some('e'),
                (false, 0x09) => Some('f'),
                (false, 0x0a) => Some('g'),
                (false, 0x0b) => Some('h'),
                (false, 0x0c) => Some('i'),
                (false, 0x0d) => Some('j'),
                (false, 0x0e) => Some('k'),
                (false, 0x0f) => Some('l'),
                (false, 0x10) => Some('m'),
                (false, 0x11) => Some('n'),
                (false, 0x12) => Some('o'),
                (false, 0x13) => Some('p'),
                (false, 0x14) => Some('q'),
                (false, 0x15) => Some('r'),
                (false, 0x16) => Some('s'),
                (false, 0x17) => Some('t'),
                (false, 0x18) => Some('u'),
                (false, 0x19) => Some('v'),
                (false, 0x1a) => Some('w'),
                (false, 0x1b) => Some('x'),
                (false, 0x1c) => Some('y'),
                (false, 0x1d) => Some('z'),
                (true, 0x04) => Some('A'),
                (true, 0x05) => Some('B'),
                (true, 0x06) => Some('C'),
                (true, 0x07) => Some('D'),
                (true, 0x08) => Some('E'),
                (true, 0x09) => Some('F'),
                (true, 0x0a) => Some('G'),
                (true, 0x0b) => Some('H'),
                (true, 0x0c) => Some('I'),
                (true, 0x0d) => Some('J'),
                (true, 0x0e) => Some('K'),
                (true, 0x0f) => Some('L'),
                (true, 0x10) => Some('M'),
                (true, 0x11) => Some('N'),
                (true, 0x12) => Some('O'),
                (true, 0x13) => Some('P'),
                (true, 0x14) => Some('Q'),
                (true, 0x15) => Some('R'),
                (true, 0x16) => Some('S'),
                (true, 0x17) => Some('T'),
                (true, 0x18) => Some('U'),
                (true, 0x19) => Some('V'),
                (true, 0x1a) => Some('W'),
                (true, 0x1b) => Some('X'),
                (true, 0x1c) => Some('Y'),
                (true, 0x1d) => Some('Z'),

                (false, 0x1E) => Some('1'),
                (false, 0x1F) => Some('2'),
                (false, 0x20) => Some('3'),
                (false, 0x21) => Some('4'),
                (false, 0x22) => Some('5'),
                (false, 0x23) => Some('6'),
                (false, 0x24) => Some('7'),
                (false, 0x25) => Some('8'),
                (false, 0x26) => Some('9'),
                (false, 0x27) => Some('0'),

                (true, 0x1E) => Some('!'),
                (true, 0x1F) => Some('@'),
                (true, 0x20) => Some('#'),
                (true, 0x21) => Some('$'),
                (true, 0x22) => Some('%'),
                (true, 0x23) => Some('^'),
                (true, 0x24) => Some('&'),
                (true, 0x25) => Some('*'),
                (true, 0x26) => Some('('),
                (true, 0x27) => Some(')'),

                (false, 0x36) => Some(','),
                (false, 0x37) => Some('.'),

                (false, 0x2A) => Some('\x08'), // backspace

                (false, 0x2C) => Some(' '),
                (false, 0x28) => Some('\n'),

                _ => None,
            }
        }
    }
    impl Driver for HidKeyboardDriver {
        fn set_endpoint(&mut self, config: &EndpointConfig) -> Result<()> {
            self.hid_driver.set_endpoint(config)
        }
        fn on_control_completed(
            &mut self,
            ep_id: EndpointId,
            setup_data: SetupData,
            buf_ptr: Option<NonNull<u8>>,
            transfered_size: usize,
        ) -> Result<TransferRequest> {
            self.hid_driver
                .on_control_completed(ep_id, setup_data, buf_ptr, transfered_size)
        }
        fn on_interrupt_completed(
            &mut self,
            ep_id: EndpointId,
            buf_ptr: NonNull<u8>,
            transfered_size: usize,
        ) -> Result<TransferRequest> {
            let req = self
                .hid_driver
                .on_interrupt_completed(ep_id, buf_ptr, transfered_size)?;

            // FIXME
            {
                const SHIFT_MASK: u8 = 0b00100010;
                let modifier = self.hid_driver.buffer()[0];
                let shift = modifier & SHIFT_MASK != 0;
                for i in 2..8 {
                    let key = self.hid_driver.buffer()[i];
                    if key == 0 || self.prev.contains(&key) {
                        continue;
                    }

                    let ch = Self::key2ascii(shift, key);
                    println!(
                        "key down: {:?} (mod: {:02x}, key: {:02x})",
                        ch, modifier, key
                    );
                }
                for key in self.prev.iter() {
                    if !self.hid_driver.buffer()[2..8].contains(&key) {
                        println!("  key up: {:02x}", key);
                    }
                }
                self.prev.copy_from_slice(&self.hid_driver.buffer()[2..8]);
            }

            Ok(req)
        }
        fn on_endpoints_configured(&mut self) -> Result<TransferRequest> {
            self.hid_driver.on_endpoints_configured()
        }
    }
}
