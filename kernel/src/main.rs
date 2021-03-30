#![no_std]
#![no_main]
#![feature(asm)]
#![feature(custom_test_frameworks)]
#![test_runner(test_runner)]
#![reexport_test_harness_main = "test_main"]

#[macro_use]
mod console;

#[macro_use]
mod utils;

mod global;
mod graphics;
mod pci;
mod sync;
mod usb;
mod x86;

use crate::graphics::{font, Color, FrameBuffer};

fn switch_ehci_to_xhci(xhc_dev: &pci::Device) {
    trace!("switch_ehci_to_xhci");

    let mut intel_ehc_exist = false;
    for dev in pci::devices() {
        let ehci = matches!(dev.as_config().read_class_code(), (0x0C, 0x03, 0x20, _));
        let vendor = dev.as_config().read_vendor_id();
        if ehci && vendor == 0x8086 {
            intel_ehc_exist = true;
            break;
        }
    }
    if !intel_ehc_exist {
        trace!("Intel EHC not found");
        return;
    }

    let superspeed_ports = xhc_dev.read_register(0x37); // USB3PRM
    xhc_dev.write_register(0x36, superspeed_ports); // USB3PRM
    let ehci2xhci_ports = xhc_dev.read_register(0x35); // XUSB2PRM
    xhc_dev.write_register(0x34, ehci2xhci_ports); // XUSB2PR
    debug!(
        "switch_ehci_to_xhci: SS={:#02x}, xHCI={:#02x}",
        superspeed_ports, ehci2xhci_ports
    );
}

#[no_mangle]
pub extern "C" fn kernel_main(frame_buffer: FrameBuffer) {
    let w = frame_buffer.width();
    let h = frame_buffer.height();
    global::init_frame_buffer(frame_buffer);

    use graphics::Render;
    global::lock_frame_buffer(|fb| fb.draw_filled_rect(0, 0, w, h, Color::BLACK));

    use font::Font;
    let (cw, ch) = font::ShinonomeFont.char_size(' ');
    let columns = (w / cw) as usize;
    let lines = (h / ch) as usize;
    global::init_console(columns, lines);

    #[cfg(test)]
    test_main();

    print!(
        r" __  __                 _            _          ___  ____  
|  \/  | __ _ _ __   __| | __ _ _ __(_)_ __    / _ \/ ___| 
| |\/| |/ _` | '_ \ / _` |/ _` | '__| | '_ \  | | | \___ \ 
| |  | | (_| | | | | (_| | (_| | |  | | | | | | |_| |___) |
|_|  |_|\__,_|_| |_|\__,_|\__,_|_|  |_|_| |_|  \___/|____/ 
"
    );
    println!();

    let scan_result = unsafe { pci::scan_all_buses() };
    debug!("scan_all_buses: {:?}", scan_result);

    for dev in pci::devices().iter() {
        trace!("{:?}", dev);
    }

    let mut xhc_dev: Option<&pci::Device> = None;
    for dev in pci::devices().iter() {
        let config = dev.as_config();
        // looking for xHC device
        if let (0x0C, 0x03, 0x30, _) = config.read_class_code() {
            xhc_dev = Some(dev);
            // prioritize intel's one
            if config.read_vendor_id() == 0x8086 {
                break;
            }
        }
    }
    match xhc_dev {
        Some(dev) => {
            if dev.as_config().read_vendor_id() == 0x8086 {
                switch_ehci_to_xhci(dev);
            }

            debug!("xHC found: {:?}", dev);
            let xhc_bar = dev.read_bar(0);
            debug!("Read BAR: {:?}", xhc_bar);
            let mmio_base = (xhc_bar.unwrap() & !0x0F) as usize;
            debug!("MMIO base: {:8x}", mmio_base);

            let mut controller = usb::xhci::Controller::new(mmio_base).unwrap();
            debug!("xHC initialized");
            controller.run();
            controller.configure_ports();

            loop {
                if let Err(e) = controller.process_event() {
                    error!("Error occurs during process_event: {:?}", e);
                }
            }

            todo!();
        }
        None => {
            error!("xHC not found");
        }
    }

    loop {
        x86::hlt();
    }
}

#[panic_handler]
#[no_mangle]
fn panic(info: &core::panic::PanicInfo) -> ! {
    #[cfg(all(test, not(feature = "test_console")))]
    x86::out16(0x501, 0x01); // exit QEMU with status 3

    error!("{}", info);
    loop {
        x86::hlt();
    }
}

#[cfg(test)]
fn test_runner(tests: &[&dyn TestCaseFn]) {
    println!("Running {} tests", tests.len());
    for test in tests {
        test.run_test();
    }
    println!("all tests passed!");
    #[cfg(not(feature = "test_console"))]
    x86::out16(0xB004, 0x2000); // exit QEMU with status 0
    loop {
        x86::hlt();
    }
}
#[cfg(test)]
trait TestCaseFn {
    fn run_test(&self);
}
#[cfg(test)]
impl<T: Fn()> TestCaseFn for T {
    fn run_test(&self) {
        print!("{} ... ", core::any::type_name::<T>());
        self();
        println!("ok!");
    }
}

#[cfg(test)]
#[test_case]
fn trivial_test() {
    assert_eq!(1 + 1, 2);
    assert_ne!(1 + 1, '田' as u32);
}
