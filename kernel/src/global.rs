use crate::console::{Attribute, Console};
use crate::graphics::{font, FrameBuffer};
use crate::sync::spin::SpinMutex;

pub static FRAME_BUF: SpinMutex<core::mem::MaybeUninit<FrameBuffer>> =
    SpinMutex::new("frame_buffer", core::mem::MaybeUninit::uninit());

pub fn init_frame_buffer(frame_buffer: FrameBuffer) {
    let mut fb = FRAME_BUF.lock();
    unsafe { fb.as_mut_ptr().write(frame_buffer) };
}

pub fn lock_frame_buffer<F: FnMut(&mut FrameBuffer)>(mut f: F) {
    let mut fb = FRAME_BUF.lock();
    let fb = unsafe { &mut *fb.as_mut_ptr() };
    f(fb)
}

pub static CONSOLE: SpinMutex<Console> = SpinMutex::new("console", Console::new(80, 27));

pub fn init_console(columns: usize, lines: usize) {
    let mut console = CONSOLE.lock();
    console.resize(columns, lines);
}

pub fn console_write(attr: Attribute, args: core::fmt::Arguments) {
    let mut console = CONSOLE.lock();

    use core::fmt::Write;
    let old_attr = console.attr;
    console.attr = attr;
    console.write_fmt(args).unwrap();
    console.attr = old_attr;

    lock_frame_buffer(|fb| {
        console.render(fb, &mut font::ShinonomeFont);
    });
}
