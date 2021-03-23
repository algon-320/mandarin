#![no_std]
#![no_main]
#![feature(llvm_asm)]

use uefi::prelude::ResultExt;

use uefi::data_types::Handle;
use uefi::proto::media::file::Directory;
use uefi::table::boot::BootServices;
use uefi::table::{Boot, SystemTable};
use uefi::Status;

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

    use uefi::proto::media::file::{
        File, FileAttribute, FileInfo, FileMode, FileType, RegularFile,
    };

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

    // Load the kernel
    let kernel: &mut [u8] = {
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

        use uefi::data_types::Align;
        assert!((buf.as_ptr() as usize) % FileInfo::alignment() == 0);

        let info: &FileInfo = kernel_elf
            .get_info::<FileInfo>(&mut buf[..])
            .unwrap_success();
        let file_size = info.file_size() as usize;

        use uefi::table::boot::AllocateType;
        use uefi::table::boot::MemoryType;
        const KERNEL_BASE_ADDR: usize = 0x100000;
        const PAGE_SIZE: usize = 0x1000;
        let page_count = (file_size + PAGE_SIZE - 1) / PAGE_SIZE;
        let page_addr = bs
            .allocate_pages(
                AllocateType::Address(KERNEL_BASE_ADDR),
                MemoryType::LOADER_DATA,
                page_count,
            )
            .unwrap_success();
        // this is safe since we own the page
        let page: &mut [u8] = unsafe {
            core::slice::from_raw_parts_mut(page_addr as *mut u8, page_count * PAGE_SIZE)
        };

        let nb = kernel_elf.read(page).unwrap_success();
        assert_eq!(nb, file_size);
        &mut page[..nb]
    };

    const ELF_ENTRY_OFFSET: isize = 24;
    let entry_addr =
        unsafe { core::ptr::read(kernel.as_ptr().offset(ELF_ENTRY_OFFSET) as *const u64) };
    let entry = unsafe { core::mem::transmute::<u64, unsafe extern "C" fn()>(entry_addr) };
    writeln!(system_table.stdout(), "entry = {:?}", entry).unwrap();

    let (_runtime, _desc_itr) = system_table
        .exit_boot_services(image_handle, &mut memmap_buf[..])
        .unwrap_success();

    unsafe { entry() };

    #[allow(clippy::empty_loop)]
    loop {}
}

/// # Safety: this function must not be called by multiple-threads simultaneously.
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
