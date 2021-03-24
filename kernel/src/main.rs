#![no_std]
#![no_main]
#![feature(llvm_asm)]

mod console;
mod global;
mod graphics;
mod sync;

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

    println!("Hello, World");
    println!();
    println!("---- intentional panic ----");
    assert_eq!(1 + 1, 3);

    loop {
        x86_hlt();
    }
}

#[panic_handler]
#[no_mangle]
fn panic(info: &core::panic::PanicInfo) -> ! {
    errorln!("{}", info);
    // x86_outw(0x0501, 0x0001); // exit QEMU with exit status 3 (= 0x01*2+1)
    loop {
        x86_hlt();
    }
}

/// halt the CPU
fn x86_hlt() {
    unsafe { llvm_asm!("hlt") };
}

/// write the word (data) to the port
#[allow(dead_code)]
fn x86_outw(port: u16, data: u16) {
    unsafe { llvm_asm!("outw $0, $1" :: "{ax}"(data), "{dx}"(port) :: "volatile") };
}
