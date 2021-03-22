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

    let mut buf = [0; 4096 * 4];
    assert!(bs.memory_map_size() < buf.len());
    let (_map_key, desc_itr) = bs.memory_map(&mut buf).unwrap_success();

    let mut root = unsafe { open_root_dir(&bs, image_handle).unwrap_success() };

    {
        use uefi::proto::media::file::{File, FileAttribute, FileMode, FileType, RegularFile};

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

    writeln!(system_table.stdout(), "done").unwrap();

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
