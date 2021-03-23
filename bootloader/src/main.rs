#![no_std]
#![no_main]
#![feature(llvm_asm)]

use uefi::prelude::ResultExt;

use uefi::data_types::Handle;
use uefi::proto::media::file::{
    Directory, File, FileAttribute, FileInfo, FileMode, FileType, RegularFile,
};
use uefi::table::{boot::BootServices, Boot, SystemTable};
use uefi::Status;

use core::slice::from_raw_parts_mut;

mod elf;

// NOTE: x86_64-unknown-uefi target seems to assume that the entry point is exposed as "efi_main"
#[no_mangle]
pub extern "C" fn efi_main(image_handle: Handle, system_table: SystemTable<Boot>) -> Status {
    use core::fmt::Write;

    writeln!(system_table.stdout(), "Hello, UEFI World!").unwrap();
    let bs = system_table.boot_services();

    let mut memmap_buf = [0; 4096 * 4];
    assert!(bs.memory_map_size() < memmap_buf.len());
    let (_map_key, desc_itr) = bs.memory_map(&mut memmap_buf).unwrap_success();

    let mut root = unsafe { open_root_dir(&bs, image_handle).unwrap_success() };

    // Collect memory maps
    {
        struct RegulerFileWriter(RegularFile);
        impl core::fmt::Write for RegulerFileWriter {
            fn write_str(&mut self, s: &str) -> core::fmt::Result {
                self.0
                    .write(s.as_bytes())
                    .map_err(|_| core::fmt::Error)?
                    .unwrap();
                Ok(())
            }
        }

        let memmap_file = root
            .open(
                "\\memmap",
                FileMode::CreateReadWrite,
                FileAttribute::empty(),
            )
            .unwrap_success();
        let mut memmap_file = match memmap_file.into_type().unwrap_success() {
            FileType::Regular(file) => RegulerFileWriter(file),
            _ => panic!("\\memmap is a directory"),
        };

        writeln!(
            memmap_file,
            "Index, Type, Type(name), PhysicalStart, NumberOfPages, Attribute"
        )
        .unwrap();

        for (i, desc) in desc_itr.enumerate() {
            writeln!(
                memmap_file,
                "{}, {:x}, {:?}, {:08x}, {}, {:x}",
                i, desc.ty.0, desc.ty, desc.phys_start, desc.page_count, desc.att
            )
            .unwrap();
        }
    }

    use uefi::proto::console::gop::GraphicsOutput;
    let gop = bs.locate_protocol::<GraphicsOutput>().unwrap_success();
    let gop = unsafe { &mut *gop.get() };
    let mode = gop.current_mode_info();
    writeln!(
        system_table.stdout(),
        "Resolution (w, h): {:?}, Pixel Format: {:?}, {} px/line",
        mode.resolution(),
        mode.pixel_format(),
        mode.stride(),
    )
    .unwrap();
    let mut fb = gop.frame_buffer();
    writeln!(
        system_table.stdout(),
        "Frame Buffer: {:p} - {:p} ({} bytes)",
        fb.as_mut_ptr(),
        fb.as_mut_ptr().wrapping_add(fb.size()),
        fb.size(),
    )
    .unwrap();

    // Load the kernel
    let kernel: &mut [u8] = {
        use uefi::data_types::Align;
        use uefi::table::boot::{AllocateType, MemoryType};

        let kernel_elf = root
            .open("\\kernel.elf", FileMode::Read, FileAttribute::empty())
            .unwrap_success();
        let mut kernel_elf = match kernel_elf.into_type().unwrap_success() {
            FileType::Regular(file) => file,
            _ => panic!("\\kernel.elf is a directory"),
        };

        // NOTE: the value was calclated by following code
        //   let sz = kernel_elf.get_info::<FileInfo>(&mut []).expect_error("");
        //   writeln!(system_table.stdout(), "required size = {:?}", sz).unwrap();
        const BUF_REQUIRED: usize = 102;
        let mut buf = [0u8; BUF_REQUIRED];

        assert!((buf.as_ptr() as usize) % FileInfo::alignment() == 0);

        let info: &FileInfo = kernel_elf
            .get_info::<FileInfo>(&mut buf[..])
            .unwrap_success();
        let file_size = info.file_size() as usize;

        // allocate a temporal buffer
        let kernel_buf: &mut [u8] = {
            let tmp = bs
                .allocate_pool(MemoryType::LOADER_DATA, file_size)
                .unwrap_success();
            unsafe { from_raw_parts_mut(tmp, file_size) }
        };

        let nb = kernel_elf.read(kernel_buf).unwrap_success();
        assert_eq!(nb, file_size);

        let kernel_ehdr: &elf::Elf64Ehdr = {
            let ehdr = kernel_buf.as_ptr() as *const elf::Elf64Ehdr;
            unsafe { &*ehdr }
        };

        let (kernel_begin, kernel_end) = kernel_ehdr.load_destination_range();
        let kernel_size = kernel_end - kernel_begin;
        writeln!(
            system_table.stdout(),
            "Kernel: {:p} - {:p}",
            kernel_begin as *const u8,
            kernel_end as *const u8,
        )
        .unwrap();

        const KERNEL_BASE_ADDR: usize = 0x100000;
        const PAGE_SIZE: usize = 0x1000;
        let page_count = (kernel_size + PAGE_SIZE - 1) / PAGE_SIZE;
        let page_addr = bs
            .allocate_pages(
                AllocateType::Address(KERNEL_BASE_ADDR),
                MemoryType::LOADER_DATA,
                page_count,
            )
            .unwrap_success();

        unsafe { kernel_ehdr.expand_load_segments() };

        // free the buffer
        bs.free_pool(kernel_buf.as_mut_ptr()).unwrap_success();

        unsafe { from_raw_parts_mut(page_addr as *mut u8, kernel_size) }
    };

    type EntryFn = fn() -> ();
    let entry = {
        const ELF_ENTRY_OFFSET: usize = 24;
        let entry_addr = kernel.as_ptr().wrapping_add(ELF_ENTRY_OFFSET) as *const u64;
        unsafe { core::mem::transmute::<u64, EntryFn>(entry_addr.read()) }
    };
    writeln!(system_table.stdout(), "entry = {:?}", entry).unwrap();

    let (_runtime, _desc_itr) = system_table
        .exit_boot_services(image_handle, &mut memmap_buf[..])
        .unwrap_success();

    entry(); // kernel_main

    #[allow(clippy::empty_loop)]
    loop {}
}

/// Safety: this function must not be called by multiple-threads simultaneously.
unsafe fn open_root_dir(bs: &BootServices, image_handle: Handle) -> uefi::Result<Directory> {
    use uefi::proto::loaded_image::LoadedImage;
    use uefi::proto::media::fs::SimpleFileSystem;

    let loaded_image = bs.handle_protocol::<LoadedImage>(image_handle)?.unwrap();
    let device = (&*loaded_image.get()).device();
    let fs = bs.handle_protocol::<SimpleFileSystem>(device)?.unwrap();
    (&mut *fs.get()).open_volume()
}

#[panic_handler]
#[no_mangle]
fn panic(_: &core::panic::PanicInfo) -> ! {
    x86_outw(0x0501, 0x0001); // exit QEMU with exit status 3 (= 0x01*2+1)
    loop {}
}

/// write the word (data) to the port
fn x86_outw(port: u16, data: u16) {
    unsafe { llvm_asm!("outw $0, $1" :: "{ax}"(data), "{dx}"(port) :: "volatile") };
}
