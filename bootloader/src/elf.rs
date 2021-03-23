#![allow(dead_code)]

use core::cmp::{max, min};
use core::ptr::copy_nonoverlapping;
use core::slice::from_raw_parts;

pub type Elf64Addr = usize;
pub type Elf64Off = u64;
pub type Elf64Half = u16;
pub type Elf64Word = u32;
pub type Elf64Sword = i32;
pub type Elf64Xword = u64;
pub type Elf64Sxword = i64;

const EI_NIDENT: usize = 16;

#[repr(C)]
pub struct Elf64Ehdr {
    pub e_ident: [u8; EI_NIDENT],
    pub e_type: Elf64Half,
    pub e_machine: Elf64Half,
    pub e_version: Elf64Word,
    pub e_entry: Elf64Addr,
    pub e_phoff: Elf64Off,
    pub e_shoff: Elf64Off,
    pub e_flags: Elf64Word,
    pub e_ehsize: Elf64Half,
    pub e_phentsize: Elf64Half,
    pub e_phnum: Elf64Half,
    pub e_shentsize: Elf64Half,
    pub e_shnum: Elf64Half,
    pub e_shstrndx: Elf64Half,
}

#[repr(u16)]
#[derive(PartialEq)]
pub enum Elf64PhdrType {
    Null = 0,
    Load = 1,
    Dynamic = 2,
    Interp = 3,
    Note = 4,
    Shlib = 5,
    Phdr = 6,
    Tls = 7,
}

#[repr(C)]
pub struct Elf64Phdr {
    pub p_type: Elf64PhdrType,
    pub p_flags: Elf64Word,
    pub p_offset: Elf64Off,
    pub p_vaddr: Elf64Addr,
    pub p_paddr: Elf64Addr,
    pub p_filesz: Elf64Xword,
    pub p_memsz: Elf64Xword,
    pub p_align: Elf64Xword,
}

impl Elf64Ehdr {
    fn phdrs(&self) -> &[Elf64Phdr] {
        let p = self as *const _ as *const u8;
        let p = p.wrapping_add(self.e_phoff as usize);
        unsafe { from_raw_parts(p as *const Elf64Phdr, self.e_phnum as usize) }
    }

    pub fn load_destination_range(&self) -> (usize, usize) {
        let mut beg = core::usize::MAX;
        let mut end = 0;
        for phdr in self
            .phdrs()
            .iter()
            .filter(|p| p.p_type == Elf64PhdrType::Load)
        {
            beg = min(beg, phdr.p_vaddr);
            end = max(end, phdr.p_vaddr + phdr.p_memsz as usize);
        }
        (beg, end)
    }

    /// Safety: destination range must be allocated in advance
    pub unsafe fn expand_load_segments(&self) {
        for phdr in self
            .phdrs()
            .iter()
            .filter(|p| p.p_type == Elf64PhdrType::Load)
        {
            let elf_begin = self as *const _ as *const u8;
            let offset = elf_begin.wrapping_add(phdr.p_offset as usize);
            let filesz = phdr.p_filesz as usize;
            copy_nonoverlapping(offset, phdr.p_vaddr as *mut u8, filesz);
            let remain = (phdr.p_memsz - phdr.p_filesz) as usize;
            core::ptr::write_bytes(phdr.p_vaddr as *mut u8, 0, remain);
        }
    }
}
