
use crate::cpu::CpuType;
use crate::external_ram::{Sram, ExternalRam};
use crate::rom::{Rom, Mbc};

pub struct MemBus {
    pub rom: Rom,
    pub wram: Vec<u8>,
    pub vram: Vec<u8>,
    pub sram: Option<Sram>,
    pub oam: Vec<u8>,
    pub io_ports: Vec<u8>,
    vram_protected: bool,
    oam_protected: bool,
    rom_bank_offset: usize,
    wram_bank_offset: usize,
    vram_bank_offset: usize,
    rom_mbc_mode: u32
}

impl MemBus {

    pub fn new(rom: Rom) -> Self {
        let sram = rom.get_sram();
        Self {
            rom,
            wram: Vec::with_capacity(8 * 4096),
            vram: Vec::with_capacity(2 * 8192),
            sram,
            oam: Vec::with_capacity(160),
            io_ports: Vec::with_capacity(256),
            vram_protected: false,
            oam_protected: false,
            rom_bank_offset: 0,
            wram_bank_offset: 0,
            vram_bank_offset: 0,
            rom_mbc_mode: 0
        }
    }

    pub fn reset(&mut self, cpu_type: CpuType) {

        if let Some(sram) = &mut self.sram {
            sram.reset();
        }

        // Clear block memory
        self.wram.fill(0);
        self.vram.fill(0);
        self.oam.fill(0);
        self.io_ports.fill(0);

        // Set various state
        self.vram_protected = false;
        self.oam_protected = false;
        self.rom_bank_offset = 0x4000;
        self.wram_bank_offset = 0x1000;
        self.vram_bank_offset = 0x0000;
        self.rom_mbc_mode = 0;

        // Initialise IO ports
        self.io_ports[5] = 0x00;
        self.io_ports[6] = 0x00;
        self.io_ports[7] = 0x00;
        self.io_ports[16] = 0x80;
        self.io_ports[17] = 0xbf;
        self.io_ports[18] = 0xf3;
        self.io_ports[20] = 0xbf;
        self.io_ports[22] = 0x3f;
        self.io_ports[23] = 0x00;
        self.io_ports[25] = 0xbf;
        self.io_ports[26] = 0x7f;
        self.io_ports[27] = 0xff;
        self.io_ports[28] = 0x9f;
        self.io_ports[30] = 0xbf;
        self.io_ports[32] = 0xff;
        self.io_ports[33] = 0x00;
        self.io_ports[34] = 0x00;
        self.io_ports[35] = 0xbf;
        self.io_ports[36] = 0x77;
        self.io_ports[37] = 0xf3;
        self.io_ports[64] = 0x91;
        self.io_ports[66] = 0x00;
        self.io_ports[67] = 0x00;
        self.io_ports[69] = 0x00;
        self.io_ports[71] = 0xfc;
        self.io_ports[72] = 0xff;
        self.io_ports[73] = 0xff;
        self.io_ports[74] = 0x00;
        self.io_ports[75] = 0x00;
        self.io_ports[85] = 0xff;
        self.io_ports[255] = 0x00;

        // Conditional state
        if cpu_type == CpuType::Sgb {
            self.io_ports[38] = 0xf0;
        } else {
            self.io_ports[38] = 0xf1;
        }
    }

    pub fn read_address(&self, address: usize) -> u8 {
        let masked_address = address & 0xffff;
        match masked_address {
            0x0000..=0x3fff => self.rom.data[masked_address],
            0x4000..=0x7fff => self.rom.data[self.rom_bank_offset + masked_address],
            0x8000..=0x9fff => self.vram[self.vram_bank_offset + masked_address - 0x8000],
            0xa000..=0xbfff => {
                if let Some(sram) = &self.sram {
                    if sram.has_timer && sram.timer_mode > 0 {
                        // TODO - Check this bit
                        sram.timer_data[sram.timer_mode as usize - 0x08]
                    } else {
                        sram.read_byte(masked_address - 0xa000)
                    }
                } else {
                    0xff
                }
            },
            0xc000..=0xcfff => self.wram[masked_address - 0xc000],
            0xd000..=0xdfff => self.wram[self.wram_bank_offset + masked_address - 0xd000],
            0xe000..=0xefff => self.wram[masked_address - 0xe000],
            0xf000..=0xfdff => self.wram[self.wram_bank_offset + masked_address - 0xf000],
            0xfe00..=0xfe9f => {
                if self.oam_protected {
                    0xff
                } else {
                    self.oam[(masked_address & 0x00ff) % 160]
                }
            },
            0xfea0..=0xfeff => 0xff,
            0xff00..=0xff7f => self.read_address(masked_address & 0x007f),
            _ => self.io_ports[masked_address & 0x00ff]
        }
    }

    pub fn write_address(&mut self, address: usize, byte: u8) {
        let masked_address = address & 0xffff;
        match masked_address {
            0x0000..=0x7fff => {
                match self.rom.mbc {
                    Mbc::None => {},
                    Mbc::Mbc1 => {
                        match masked_address {
                            0x0000..=0x1fff => {
                                // Only 4 bits are used. Writing 0xa enables SRAM, anything else disables it
                                if let Some(sram) = &mut self.sram {
                                    sram.enable_flag = (byte & 0x0f) == 0x0a;
                                }
                            },
                            0x2000..=0x3fff => {
                                // Set low 5 bits of bank number
                                self.rom_bank_offset &= 0xfff80000;
                                let mut byte = byte & 0x1f;
                                if byte == 0x00 {
                                    byte = 0x01;
                                }
                                self.rom_bank_offset |= byte as usize * 0x4000;
                                self.rom_bank_offset &= self.rom.bank_select_mask as usize * 0x4000;
                            },
                            0x4000..=0x5fff => {
                                let byte = byte & 0x03;
                                if let Some(sram) = &mut self.sram {
                                    if self.rom_mbc_mode != 0 {
                                        sram.bank_offset = byte as usize * 0x2000; // Select RAM bank
                                    } else {
                                        self.rom_bank_offset &= 0xffe7c000;
                                        self.rom_bank_offset |= byte as usize * 0x80000;
                                        self.rom_bank_offset &= self.rom.bank_select_mask as usize * 0x4000;
                                    }
                                } else {
                                    // TODO - Check for anything needed here
                                }
                            },
                            _ => {
                                self.rom_mbc_mode = 0;
                                if let Some(sram) = &self.sram {
                                    if sram.size_bytes > 8192 {
                                        self.rom_mbc_mode = byte as u32 & 0x01;
                                    }
                                }
                            }
                        }
                    },
                    Mbc::Mbc2 => {
                        match masked_address {
                            0x0000..=0x0fff => {
                                if let Some(sram) = &mut self.sram {
                                    // Only 4 bits are used. Writing 0xa enables SRAM.
                                    sram.enable_flag = (byte & 0x0f) == 0x0a;
                                }
                            },
                            0x2100..=0x21fe => {
                                let mut byte = (byte & 0x0f) & self.rom.bank_select_mask as u8;
                                if byte == 0x00 {
                                    byte = 0x01;
                                }
                                self.rom_bank_offset = byte as usize * 0x4000;
                            },
                            _ => {}
                        }
                    },
                    Mbc::Mbc3 => {
                        match masked_address {
                            0x0000..=0x1fff => {
                                if let Some(sram) = &mut self.sram {
                                    sram.enable_flag = (byte & 0x0f) == 0x0a; // Also enables timer registers
                                }
                            },
                            0x2000..=0x3fff => {
                                let mut byte = byte & self.rom.bank_select_mask as u8;
                                if byte == 0x00 {
                                    byte = 0x01;
                                }
                                self.rom_bank_offset = byte as usize * 0x4000;
                            },
                            0x4000..=0x5fff => {
                                if let Some(sram) = &mut self.sram {
                                    let byte = byte & 0x0f;
                                    if byte < 0x04 {
                                        sram.bank_offset = byte as usize * 0x2000;
                                        sram.timer_mode = 0;
                                    } else if (byte >= 0x08) && (byte < 0x0d) {
                                        sram.timer_mode = byte as u32;
                                    } else {
                                        sram.timer_mode = 0;
                                    }
                                }
                            },
                            _ => {
                                if let Some(sram) = &mut self.sram {
                                    let byte = byte & 0x01;
                                    if (sram.timer_latch == 0x00) && (byte == 0x01) {
                                        sram.latch_timer_data();
                                    }
                                    sram.timer_latch = byte as u32;
                                }
                            }
                        }
                    },
                    Mbc::Mbc5 => {
                        match masked_address {
                            0x0000..=0x1fff => {
                                // RAMG - 4 bits, enable external RAM by writing 0xa
                                if let Some(sram) = &mut self.sram {
                                    sram.enable_flag = (byte & 0x0f) == 0x0a;
                                }
                            },
                            0x2000..=0x2fff => {
                                // ROMB0 - lower 8 bits of 9-bit ROM bank (note MBC5 can select bank 0 here)
                                let masked_byte = byte as usize & self.rom.bank_select_mask as usize;
                                self.rom_bank_offset = (self.rom_bank_offset & 0x00400000) | (masked_byte * 0x4000);
                            },
                            0x3000..=0x3fff => {
                                // ROMB1 - 1 bit, upper bit of 9-bit RAM bank (note MBC5 can select bank 0 here)
                                self.rom_bank_offset &= 0x003fc000;
                                if (byte & 0x01) != 0x00 {
                                    self.rom_bank_offset |= 0x00004000;
                                }
                                self.rom_bank_offset &= self.rom.bank_select_mask as usize * 0x4000;
                            },
                            0x4000..=0x5fff => {
                                // RAMB - 4-bit RAM bank
                                if let Some(sram) = &mut self.sram {
                                    sram.bank_offset = (byte & 0x0f) as usize * 0x2000;
                                }
                            },
                            _ => {} // Writing to 0x6000 - 0x7fff does nothing
                        }
                    }
                }
            },
            0x8000..=0x9fff => {
                if self.vram_protected {
                    return;
                }

                // Mask to address within range 0x0000-0x1fff and write to that VRAM address
                let masked_address = masked_address & 0x1fff;
                self.vram[self.vram_bank_offset + masked_address] = byte;

                // Decode character set from GB format to something more computer-friendly, only if writing within tileset
                if masked_address >= 0x1800 {
                    return;
                }

                // Get the pair of bytes just modified (i.e. one row in the character)
                // Note there are 384 characters in the map, per VRAM bank, each stored with 16 bytes
                let relative_vram_address = masked_address & 0x1ffe;
                let byte1 = self.vram[self.vram_bank_offset + relative_vram_address] as u32 & 0x000000ff;
                let byte2 = self.vram[self.vram_bank_offset + relative_vram_address + 1] as u32 & 0x000000ff;


                // Find the address into decoded data to update now
                // The output format uses 64 bytes per tile rather than 16, hence input address * 4
                let mut output_address = relative_vram_address * 4;
                if self.vram_bank_offset != 0 {
                    output_address += 24576;
                }

                // TODO - Write this to tile set cache in GPU
                // self.tile_set[output_address] = ((byte2 >> 6) & 0x02) + (byte1 >> 7);
                // self.tile_set[output_address + 1] = ((byte2 >> 5) & 0x02) + ((byte1 >> 6) & 0x01);
                // self.tile_set[output_address + 2] = ((byte2 >> 4) & 0x02) + ((byte1 >> 5) & 0x01);
                // self.tile_set[output_address + 3] = ((byte2 >> 3) & 0x02) + ((byte1 >> 4) & 0x01);
                // self.tile_set[output_address + 4] = ((byte2 >> 2) & 0x02) + ((byte1 >> 3) & 0x01);
                // self.tile_set[output_address + 5] = ((byte2 >> 1) & 0x02) + ((byte1 >> 2) & 0x01);
                // self.tile_set[output_address + 6] = (byte2 & 0x02) + ((byte1 >> 1) & 0x01);
                // self.tile_set[output_address + 7] = ((byte2 << 1) & 0x02) + (byte1 & 0x01);
            },
            0xa000..=0xbfff => {
                if let Some(sram) = &mut self.sram {
                    if sram.enable_flag {
                        if sram.has_timer {
                            if sram.timer_mode > 0 {
                                sram.latch_timer_data();
                                sram.write_timer_data(sram.timer_mode as usize, byte);
                                sram.timer_data[sram.timer_mode as usize] = byte;  // TODO - Check this
                            } else if sram.bank_offset < 0x8000 {
                                sram.write_byte(masked_address, byte);
                            }
                        } else {
                            let byte = if self.rom.mbc == Mbc::Mbc2 {
                                byte & 0x0f
                            } else {
                                byte
                            };
                            sram.write_byte(masked_address, byte);
                        }
                    }
                }
            },
            0xc000..=0xcfff => {
                self.wram[masked_address & 0x0fff] = byte;
            },
            0xd000..=0xdfff => {
                self.wram[self.wram_bank_offset + (masked_address & 0x0fff)] = byte;
            },
            0xe000..=0xefff => {
                self.wram[masked_address & 0x0fff] = byte;
            },
            0xf000..=0xfdff => {
                self.wram[self.wram_bank_offset + (masked_address & 0x0fff)] = byte;
            },
            0xfe00..=0xfe9f => {
                if !self.oam_protected {
                    self.oam[(masked_address & 0x00ff) % 160] = byte;
                }
            },
            0xfea0..=0xfeff => {}, // Unusable
            0xff00..=0xff7f => {
                self.write_io(masked_address & 0x007f, byte);
            },
            _ => {
                self.io_ports[masked_address & 0x00ff] = byte;
            }
        }
    }

    pub fn read_address_16(&self, address: usize) -> (u8, u8) {
        todo!()
    }

    pub fn write_address_16(&mut self, address: usize, bytes: (u8, u8)) {
        todo!()
    }

    pub fn read_io(&self, address: usize) -> u8 {
        todo!()
    }

    pub fn write_io(&mut self, address: usize, byte: u8) {

    }

    pub fn set_vram_protection(&mut self, protected: bool) {
        self.vram_protected = protected;
    }

    pub fn set_oam_protection(&mut self, protected: bool) {
        self.oam_protected = protected;
    }

    pub fn perform_and(&mut self, address: usize, byte: u8) {
        self.write_address(address, self.read_address(address) & byte);
    }

    pub fn perform_or(&mut self, address: usize, byte: u8) {
        self.write_address(address, self.read_address(address) | byte);
    }
}
