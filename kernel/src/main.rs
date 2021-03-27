#![no_std]
#![no_main]
#![feature(asm)]
#![feature(custom_test_frameworks)]
#![test_runner(test_runner)]
#![reexport_test_harness_main = "test_main"]

mod console;
mod global;
mod graphics;
mod sync;

mod x86 {
    /// halt the CPU
    pub fn hlt() {
        unsafe { asm!("hlt") };
    }

    #[inline]
    pub fn out16(port: u16, data: u16) {
        unsafe {
            asm!(
                "out dx, ax",
                in("dx") port,
                in("ax") data,
                options(nomem, nostack)
            )
        };
    }
    #[inline]
    pub fn in16(port: u16) -> u16 {
        let ax: u16;
        unsafe {
            asm!(
                "in ax, dx",
                out("ax") ax,
                in("dx") port,
                options(nomem, nostack)
            )
        };
        ax
    }

    #[inline]
    pub fn out32(port: u16, data: u32) {
        unsafe {
            asm!(
                "out dx, eax",
                in("dx") port,
                in("eax") data,
                options(nomem, nostack)
            )
        };
    }
    #[inline]
    pub fn in32(port: u16) -> u32 {
        let eax: u32;
        unsafe {
            asm!(
                "in eax, dx",
                out("eax") eax,
                in("dx") port,
                options(nomem, nostack)
            )
        };
        eax
    }
}

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
    println!("---- intentional panic ----");
    assert_eq!(1 + 1, 3);

    loop {
        x86::hlt();
    }
}

#[panic_handler]
#[no_mangle]
fn panic(info: &core::panic::PanicInfo) -> ! {
    #[cfg(test)]
    x86::out16(0x501, 0x01); // exit QEMU with status 3

    errorln!("{}", info);
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
