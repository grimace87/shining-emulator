
#[derive(Default)]
pub struct Rom {
    pub valid: bool,
    rom: Vec<u8>,
    pub name: String,
    pub cgb_flag: bool,
    pub sgb_flag: bool,
    pub has_sram: bool,
    pub has_battery: bool,
    pub has_timer: bool,
    pub has_rumble: bool,
    pub mbc: Mbc,
    pub size_bytes: u32,
    bank_select_mask: u32,
    pub sram_size_bytes: u32
}

#[derive(PartialEq)]
pub enum Mbc {
    None,
    Mbc1,
    Mbc2,
    Mbc3,
    Mbc5
}

impl Default for Mbc {
    fn default() -> Self {
        Mbc::None
    }
}

const OFFICIAL_HEADER: [u8; 48] = [
    0xce, 0xed, 0x66, 0x66, 0xcc, 0x0d, 0x00, 0x0b, 0x03, 0x73, 0x00, 0x83, 0x00, 0x0c, 0x00, 0x0d,
    0x00, 0x08, 0x11, 0x1f, 0x88, 0x89, 0x00, 0x0e, 0xdc, 0xcc, 0x6e, 0xe6, 0xdd, 0xdd, 0xd9, 0x99,
    0xbb, 0xbb, 0x67, 0x63, 0x6e, 0x0e, 0xec, 0xcc, 0xdd, 0xdc, 0x99, 0x9f, 0xbb, 0xb9, 0x33, 0x3e
];

impl Rom {

    pub fn new_from_data(data: Vec<u8>) -> Self {

        // Check data meets minimum size
        if data.len() < 32768 {
            eprintln!("Cannot load ROM info from only {} bytes", data.len());
            return Self {
                valid: false,
                ..Self::default()
            }
        }

        // Check ROM header; print to output if not valid
        let header_slice = &data[0x0104..0x0134];
        let mut mismatch_count = 0;
        for (i, header_byte) in OFFICIAL_HEADER.iter().enumerate() {
            if *header_byte != header_slice[i] {
                mismatch_count += 1;
            }
        }
        if mismatch_count > 1 {
            eprintln!("Header incorrect in {} of 48 bytes", mismatch_count);
        }

        // Get colour compatibility and name string
        let cgb_byte = data[0x0143];
        let cgb_flag = cgb_byte == 0x80 || cgb_byte == 0xc0;
        let name_string = String::from(
            if cgb_flag {
                std::str::from_utf8(&data[0x0134..0x0143]).unwrap()
            } else {
                std::str::from_utf8(&data[0x0134..0x0143]).unwrap()
            }
        );

        // Next flags in header - SGB, cartridge type
        let sgb_flag = data[0x0146] == 0x03;
        let cartridge_type = data[0x0147];

        // Get MBC and other properties based on values above
        let mut has_sram = false;
        let mut has_battery = false;
        let mut has_timer = false;
        let mut has_rumble = false;
        let mbc = match cartridge_type {

            // No MBC
            0x00 => {
                Mbc::None
            },
            0x08 => {
                has_sram = true;
                Mbc::None
            },
            0x09 => {
                has_sram = true;
                has_battery = true;
                Mbc::None
            },

            // MBC1
            0x01 => {
                Mbc::Mbc1
            },
            0x02 => {
                has_sram = true;
                Mbc::Mbc1
            },
            0x03 => {
                has_sram = true;
                has_battery = true;
                Mbc::Mbc1
            },

            // MBC2
            0x05 => {
                has_sram = true;
                Mbc::Mbc2
            },
            0x06 => {
                has_sram = true;
                has_battery = true;
                Mbc::Mbc2
            },

            // MBC3
            0x0f => {
                has_battery = true;
                has_timer = true;
                Mbc::Mbc3
            },
            0x10 => {
                has_sram = true;
                has_battery = true;
                has_timer = true;
                Mbc::Mbc3
            },
            0x11 => {
                Mbc::Mbc3
            },
            0x12 => {
                has_sram = true;
                Mbc::Mbc3
            },
            0x13 => {
                has_sram = true;
                has_battery = true;
                Mbc::Mbc3
            },

            // MBC5
            0x19 => {
                Mbc::Mbc5
            },
            0x1a => {
                has_sram = true;
                Mbc::Mbc5
            },
            0x1b => {
                has_sram = true;
                has_battery = true;
                Mbc::Mbc5
            },
            0x1c => {
                has_rumble = true;
                Mbc::Mbc5
            },
            0x1d => {
                has_sram = true;
                has_rumble = true;
                Mbc::Mbc5
            },
            0x1e => {
                has_sram = true;
                has_battery = true;
                has_rumble = true;
                Mbc::Mbc5
            },

            // Unsupported types: MBC6, MBC7, MMM01, HuC-1, HuC-3, TAMA5, Pocket Camera

            _ => {
                eprintln!("Unsupported cartridge type: {}", cartridge_type);
                return Self {
                    valid: false,
                    ..Self::default()
                };
            }
        };

        // ROM size flag
        let size_enum = data[0x0148];
        let (size_bytes, bank_select_mask): (u32, u32) = match size_enum {
            0x00 => (32768, 0x00),
            0x01 => (65536, 0x03),
            0x02 => (131072, 0x07),
            0x03 => (262144, 0x0f),
            0x04 => (524288, 0x1f),
            0x05 => (1048576, 0x3f),
            0x06 => (2097152, 0x7f),
            0x07 => (4194304, 0xff),
            0x08 => (8388608, 0x1ff),
            0x52 => (1179648, 0x7f),
            0x53 => (1310720, 0x7f),
            0x54 => (1572864, 0x7f),
            _ => {
                eprintln!("Unsupported ROM size type: {}", size_enum);
                return Self {
                    valid: false,
                    ..Self::default()
                };
            },
        };

        // SRAM size flag
        let sram_size_enum = data[0x0149];
        let sram_size_bytes = match sram_size_enum {
            0x00 => {
                if mbc == Mbc::Mbc2 {
                    512
                } else {
                    0
                }
            },
            0x01 => 2048,
            0x02 => 8192,
            0x03 => 32768,
            0x04 => 131072,
            0x05 => 65536,
            _ => {
                eprintln!("Unsupported SRAM size type: {}", sram_size_enum);
                return Self {
                    valid: false,
                    ..Self::default()
                };
            }
        };

        // TODO: if has_battery then open the backing file

        // Check the checksum value; print message if it fails
        // 0x4a - 0x4c contain a japanese designation, an old licensee number and a version number
        // 0x4d is the header checksum. Game won't work if it checks incorrectly
        let checksum = data[0x014d];
        let mut sum: usize = 0;
        for byte in &data[0x0134..0x014d] {
            sum = sum.wrapping_sub(*byte as usize + 1);
        }
        if sum % 256 != checksum as usize {
            eprintln!("Checksum mismatch");
        }

        // Copy ROM data into new Vec
        let mut rom: Vec<u8> = vec![0; 256 * 16384];
        let mut rom_slice = &mut rom[0..(size_bytes as usize)];
        rom_slice.copy_from_slice(&data[0..(size_bytes as usize)]);

        Self {
            valid: true,
            rom,
            name: name_string,
            cgb_flag,
            sgb_flag,
            has_sram,
            has_battery,
            has_timer,
            has_rumble,
            mbc,
            size_bytes,
            bank_select_mask,
            sram_size_bytes
        }
    }
}
