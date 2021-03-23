#![no_std]
#![no_main]
#![feature(llvm_asm)]

#[repr(u8)]
pub enum PixelFormat {
    Rgb,
    Bgr,
    BitMask,
    BltOnly,
}
#[repr(C)]
pub struct FrameBuffer {
    buffer: *mut u8,
    resolution_horizontal: u64,
    resolution_vertical: u64,
    stride: u64,
    format: PixelFormat,
}

#[no_mangle]
pub extern "C" fn kernel_main(mut frame_buffer: FrameBuffer) {
    let h = frame_buffer.resolution_vertical as usize;
    let w = frame_buffer.resolution_horizontal as usize;

    loop {
        x86_hlt();
    }
}

#[panic_handler]
#[no_mangle]
fn panic(_: &core::panic::PanicInfo) -> ! {
    x86_outw(0x0501, 0x0001); // exit QEMU with exit status 3 (= 0x01*2+1)
    loop {
        x86_hlt();
    }
}

/// halt the CPU
fn x86_hlt() {
    unsafe { llvm_asm!("hlt") };
}

/// write the word (data) to the port
fn x86_outw(port: u16, data: u16) {
    unsafe { llvm_asm!("outw $0, $1" :: "{ax}"(data), "{dx}"(port) :: "volatile") };
}
