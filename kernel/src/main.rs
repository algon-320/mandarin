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

#[derive(Clone, Copy)]
pub struct Color {
    r: u8,
    g: u8,
    b: u8,
}
#[allow(dead_code)]
impl Color {
    const WHITE: Self = Self {
        r: 0xFF,
        g: 0xFF,
        b: 0xFF,
    };
    const BLACK: Self = Self {
        r: 0x00,
        g: 0x00,
        b: 0x00,
    };
    const RED: Self = Self {
        r: 0xFF,
        g: 0x00,
        b: 0x00,
    };
    const GREEN: Self = Self {
        r: 0x00,
        g: 0xFF,
        b: 0x00,
    };
    const BLUE: Self = Self {
        r: 0x00,
        g: 0x00,
        b: 0xFF,
    };
}

trait PixelWriter {
    fn write_pixel(&mut self, x: usize, y: usize, c: Color);

    fn write_rect(&mut self, x: usize, y: usize, w: usize, h: usize, c: Color) {
        for iy in 0..h {
            for ix in 0..w {
                self.write_pixel(x + ix, y + iy, c);
            }
        }
    }

    fn write_char(&mut self, x: usize, y: usize, c: Color, ch: char) {
        const SHINONOME_FONT: &[u8] = include_bytes!("../assets/hankaku.bin");
        let mut ch = ch as usize;
        if SHINONOME_FONT.len() < ch {
            ch = b'?' as usize;
        }
        for iy in 0..16 {
            let row = SHINONOME_FONT[ch * 16 + iy];
            for ix in 0..8 {
                if row & (0x80 >> ix) != 0 {
                    self.write_pixel(x + ix, y + iy, c);
                }
            }
        }
    }
    fn write_str(&mut self, x: usize, y: usize, color: Color, s: &str) {
        let mut ix = 0;
        for c in s.chars() {
            self.write_char(x + ix, y, color, c);
            ix += 10;
        }
    }
}

impl PixelWriter for FrameBuffer {
    fn write_pixel(&mut self, x: usize, y: usize, Color { r, g, b }: Color) {
        let idx = self.stride as usize * y + x;
        let p = unsafe { self.buffer.add(4 * idx) };
        let data: [u8; 4] = match self.format {
            PixelFormat::Rgb => [r, g, b, 0],
            PixelFormat::Bgr => [b, g, r, 0],
            _ => unimplemented!(),
        };
        for (i, &v) in data.iter().enumerate() {
            unsafe { p.add(i).write_volatile(v) };
        }
    }
}

#[no_mangle]
pub extern "C" fn kernel_main(mut frame_buffer: FrameBuffer) {
    let h = frame_buffer.resolution_vertical as usize;
    let w = frame_buffer.resolution_horizontal as usize;

    frame_buffer.write_rect(0, 0, w, h, Color::WHITE);
    frame_buffer.write_str(50, 50, Color::BLACK, "Hello, World!");
    frame_buffer.write_str(50, 70, Color::RED, "Written in pure Rust (so far)");

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
