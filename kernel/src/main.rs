#![no_std]
#![no_main]
#![feature(asm)]
#![feature(custom_test_frameworks)]
#![test_runner(test_runner)]
#![reexport_test_harness_main = "test_main"]

#[macro_use]
mod console;
mod global;
mod graphics;
mod pci;
mod sync;
mod utils;
mod x86;

use crate::graphics::{font, Color, FrameBuffer};

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

    println!("Hello, World");
    println!();

    let scan_result = unsafe { pci::scan_all_buses() };
    debug!("scan_all_buses: {:?}", scan_result);

    for dev in pci::devices().iter() {
        trace!("{:?}", dev);
    }

    loop {
        x86::hlt();
    }
}

#[panic_handler]
#[no_mangle]
fn panic(info: &core::panic::PanicInfo) -> ! {
    #[cfg(test)]
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
    x86::out16(0xB004, 0x2000); // exit QEMU with status 0
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
