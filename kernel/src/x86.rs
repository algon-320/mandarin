#![allow(dead_code)]

/// halt the CPU
#[inline]
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
