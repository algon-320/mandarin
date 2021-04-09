#[derive(Debug)]
pub enum Error {
    Full,
}
impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Full => {
                write!(f, "More than {} devices found", MAX_DEVICES)
            }
        }
    }
}

pub type Result<T> = core::result::Result<T, Error>;

const CONFIG_ADDR: u16 = 0x0CF8;
const CONFIG_DATA: u16 = 0x0CFC;

use crate::x86::{in32, out32};

fn write_addr(addr: u32) {
    out32(CONFIG_ADDR, addr);
}

fn write_data(data: u32) {
    out32(CONFIG_DATA, data);
}

fn read_data() -> u32 {
    in32(CONFIG_DATA)
}

#[derive(Clone, Copy)]
pub struct Config {
    pub bus: u8,
    pub device: u8,
    pub function: u8,
}
impl Config {
    pub fn make_addr(&self, reg_idx: u8) -> u32 {
        let enable = 1;
        fn shl(x: u8, w: u32) -> u32 {
            let (r, of) = (x as u32).overflowing_shl(w);
            assert!(!of);
            r
        }
        let (reg_addr, of) = reg_idx.overflowing_shl(2);
        assert!(!of);
        shl(enable, 31)
            | shl(self.bus, 16)
            | shl(self.device, 11)
            | shl(self.function, 8)
            | shl(reg_addr, 0)
    }

    pub fn read_vendor_id(&self) -> u16 {
        write_addr(self.make_addr(0));
        (read_data() & 0x0000FFFF) as u16
    }

    fn read_device_id(&self) -> u16 {
        write_addr(self.make_addr(0));
        ((read_data() & 0xFFFF0000) >> 16) as u16
    }

    /// (Base Class, Sub Class, Interface, Revision ID)
    pub fn read_class_code(&self) -> (u8, u8, u8, u8) {
        write_addr(self.make_addr(2));
        let code = read_data() as u32;
        let base = ((code & 0xFF000000) >> 24) as u8;
        let sub_ = ((code & 0x00FF0000) >> 16) as u8;
        let ifce = ((code & 0x0000FF00) >> 8) as u8;
        let rvid = ((code & 0x000000FF) >> 0) as u8;
        (base, sub_, ifce, rvid)
    }

    fn read_header_type(&self) -> u8 {
        write_addr(self.make_addr(3));
        ((read_data() & 0x00FF0000) >> 16) as u8
    }

    fn read_bus_number(&self) -> u32 {
        write_addr(self.make_addr(6));
        read_data()
    }
}

#[derive(Clone)]
pub struct Device {
    bus: u8,
    device: u8,
    function: u8,
    header_type: u8,
}
impl From<Config> for Device {
    fn from(config: Config) -> Self {
        let Config {
            bus,
            device,
            function,
        } = config;
        Self {
            bus,
            device,
            function,
            header_type: config.read_header_type(),
        }
    }
}
impl Device {
    pub fn as_config(&self) -> Config {
        Config {
            bus: self.bus,
            device: self.device,
            function: self.function,
        }
    }

    pub fn read_register(&self, reg_idx: u8) -> u32 {
        write_addr(self.as_config().make_addr(reg_idx));
        read_data()
    }
    pub fn write_register(&self, reg_idx: u8, value: u32) {
        write_addr(self.as_config().make_addr(reg_idx));
        write_data(value);
    }

    pub fn read_bar(&self, bar_idx: u8) -> Option<u64> {
        if bar_idx >= 6 {
            return None;
        }

        const BAR_OFFSET: u8 = 4;
        let bar = self.read_register(BAR_OFFSET + bar_idx);

        // 32-bit address
        if bar & 0b100 == 0 {
            return Some(bar as u64);
        }

        // 64-bit address
        if bar_idx >= 5 {
            return None;
        }

        let bar_upper = self.read_register(BAR_OFFSET + bar_idx + 1);
        Some(((bar_upper as u64) << 32) | bar as u64)
    }
}
impl core::fmt::Debug for Device {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let vendor_id = self.as_config().read_vendor_id();
        let (base, sub, iface, rev_id) = self.as_config().read_class_code();
        let class_code =
            (base as u32) << 24 | (sub as u32) << 16 | (iface as u32) << 8 | rev_id as u32;
        write!(
            f,
            "{}.{}.{}: vendor {:04x}, class {:08x}, head {:02x}",
            self.bus, self.device, self.function, vendor_id, class_code, self.header_type
        )
    }
}

use crate::utils::FixedVec;
const MAX_DEVICES: usize = 32;
static mut DEVICES: FixedVec<Device, MAX_DEVICES> = FixedVec::new();
pub fn devices() -> &'static [Device] {
    unsafe { DEVICES.as_slice() }
}

fn is_single_function_device(header_type: u8) -> bool {
    (header_type & 0x80) == 0
}

fn add_device(config: Config) -> Result<()> {
    let device: Device = config.into();
    if unsafe { DEVICES.push(device) }.is_none() {
        Err(Error::Full)
    } else {
        Ok(())
    }
}

fn scan_function(config: Config) -> Result<()> {
    add_device(config)?;

    let (base, sub, _, _) = config.read_class_code();
    if base == 0x06 && sub == 0x04 {
        // if this device is a PCI-to-PIC bridge.
        let bus_number = config.read_bus_number();
        let secondary = ((bus_number & 0x0000FF00) >> 8) as u8;
        scan_bus(Config {
            bus: secondary,
            device: 0,
            function: 0,
        })
    } else {
        Ok(())
    }
}

fn scan_device(mut config: Config) -> Result<()> {
    config.function = 0;
    scan_function(config)?;
    if is_single_function_device(config.read_header_type()) {
        Ok(())
    } else {
        for func in 1..8 {
            config.function = func;
            if config.read_vendor_id() == 0xFFFF {
                continue;
            }
            scan_function(config)?;
        }
        Ok(())
    }
}

fn scan_bus(config: Config) -> Result<()> {
    for device in 0..32 {
        let dev = Config {
            bus: config.bus,
            device,
            function: 0,
        };
        if dev.read_vendor_id() == 0xFFFF {
            continue;
        }
        scan_device(dev)?;
    }
    Ok(())
}

/// Safety: this function must not be called more than once.
pub unsafe fn scan_all_buses() -> Result<()> {
    let host_bridge = Config {
        bus: 0,
        device: 0,
        function: 0,
    };
    if is_single_function_device(host_bridge.read_header_type()) {
        scan_bus(host_bridge)
    } else {
        for function in 1..8 {
            let another = Config {
                function,
                ..host_bridge
            };
            if another.read_vendor_id() == 0xFFFF {
                continue;
            }
            scan_bus(Config {
                bus: function,
                device: 0,
                function: 0,
            })?;
        }
        Ok(())
    }
}
