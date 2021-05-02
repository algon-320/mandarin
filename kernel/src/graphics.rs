#[allow(dead_code)]
#[derive(Debug)]
#[repr(u8)]
pub enum PixelFormat {
    Rgb,
    Bgr,
    BitMask,
    BltOnly,
}

#[derive(Debug)]
#[repr(C)]
pub struct FrameBuffer {
    buffer: *mut u8,
    resolution_horizontal: u64,
    resolution_vertical: u64,
    stride: u64,
    format: PixelFormat,
}

unsafe impl Send for FrameBuffer {}

impl FrameBuffer {
    pub fn width(&self) -> isize {
        self.resolution_horizontal as isize
    }
    pub fn height(&self) -> isize {
        self.resolution_vertical as isize
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}
#[allow(dead_code)]
impl Color {
    pub const WHITE: Self = Self {
        r: 0xFF,
        g: 0xFF,
        b: 0xFF,
    };
    pub const BLACK: Self = Self {
        r: 0x00,
        g: 0x00,
        b: 0x00,
    };
    pub const RED: Self = Self {
        r: 0xFF,
        g: 0x00,
        b: 0x00,
    };
    pub const GREEN: Self = Self {
        r: 0x00,
        g: 0xFF,
        b: 0x00,
    };
    pub const BLUE: Self = Self {
        r: 0x00,
        g: 0x00,
        b: 0xFF,
    };
}

pub trait Render {
    fn draw_pixel(&mut self, x: isize, y: isize, c: Color);

    fn draw_filled_rect(&mut self, x: isize, y: isize, w: isize, h: isize, c: Color) {
        for iy in 0..h {
            for ix in 0..w {
                self.draw_pixel(x + ix, y + iy, c);
            }
        }
    }
}

impl Render for FrameBuffer {
    fn draw_pixel(&mut self, x: isize, y: isize, Color { r, g, b }: Color) {
        {
            let w = self.resolution_horizontal as isize;
            let h = self.resolution_vertical as isize;
            if !(0 <= x && x < w && 0 <= y && y < h) {
                return;
            }
        }

        debug_assert!(!self.buffer.is_null());

        let data: [u8; 4] = match self.format {
            PixelFormat::Rgb => [r, g, b, 0],
            PixelFormat::Bgr => [b, g, r, 0],
            _ => unimplemented!(),
        };

        let (x, y) = (x as usize, y as usize);
        let idx = self.stride as usize * y + x;
        let p = unsafe { self.buffer.add(4 * idx) };
        for (i, &v) in data.iter().enumerate() {
            unsafe { p.add(i).write_volatile(v) };
        }
    }
}

pub struct Scaled<R: Render, const SCALE: isize>(pub R);
impl<R: Render, const SCALE: isize> Render for Scaled<R, SCALE> {
    fn draw_pixel(&mut self, x: isize, y: isize, color: Color) {
        self.0
            .draw_filled_rect(x * SCALE, y * SCALE, SCALE, SCALE, color);
    }
}

pub mod font {
    use super::{Color, Render};

    pub trait Font {
        fn char_size(&mut self, ch: char) -> (isize, isize);
        fn draw_char<R: Render>(
            &mut self,
            r: &mut R,
            x: isize,
            y: isize,
            fg: Color,
            bg: Option<Color>,
            ch: char,
        );
        fn draw_str<R: Render>(
            &mut self,
            r: &mut R,
            x: isize,
            y: isize,
            fg: Color,
            bg: Option<Color>,
            s: &str,
        );
    }

    pub struct ShinonomeFont;
    impl Font for ShinonomeFont {
        fn char_size(&mut self, _ch: char) -> (isize, isize) {
            (8 + 2, 16 + 2) // monospaced
        }
        fn draw_char<R: Render>(
            &mut self,
            renderer: &mut R,
            x: isize,
            y: isize,
            fg: Color,
            bg: Option<Color>,
            ch: char,
        ) {
            const SHINONOME_FONT: &[u8] = include_bytes!("../assets/hankaku.bin");
            let mut ch = ch as usize;
            if SHINONOME_FONT.len() < ch {
                ch = b'?' as usize;
            }
            for iy in 0..16 {
                let row = SHINONOME_FONT[ch * 16 + iy];
                for ix in 0..8 {
                    if row & (0x80 >> ix) != 0 {
                        renderer.draw_pixel(1 + x + ix as isize, 1 + y + iy as isize, fg);
                    } else if let Some(bg) = bg {
                        renderer.draw_pixel(1 + x + ix as isize, 1 + y + iy as isize, bg);
                    }
                }
            }
        }
        fn draw_str<R: Render>(
            &mut self,
            renderer: &mut R,
            x: isize,
            y: isize,
            fg: Color,
            bg: Option<Color>,
            s: &str,
        ) {
            let mut ix = 0;
            for c in s.chars() {
                self.draw_char(renderer, x + ix, y, fg, bg, c);
                ix += 10;
            }
        }
    }
}
