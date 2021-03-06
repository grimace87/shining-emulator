
use crate::emulator::mem::MemoryMap;
use crate::emulator::rom::Rom;
use crate::emulator::external_ram::Sram;

pub struct MemBus {
    pub rom: Rom,
    pub wram: Vec<u8>,
    pub vram: Vec<u8>,
    pub sram: Option<Sram>,
    pub io_ports: Vec<u8>,
    pub access_oam: bool,
    pub access_vram: bool,
    rom_bank_offset: usize,
    wram_bank_offset: usize,
    vram_bank_offset: usize
}

impl MemBus {

    pub fn new(rom: Rom) -> Self {
        let sram = rom.get_sram();
        Self {
            rom,
            wram: Vec::with_capacity(8 * 4096),
            vram: Vec::with_capacity(2 * 8192),
            sram,
            io_ports: Vec::with_capacity(256),
            access_oam: false,
            access_vram: false,
            rom_bank_offset: 0,
            wram_bank_offset: 0,
            vram_bank_offset: 0
        }
    }

    pub fn reset(&mut self, sgb_flag: bool) {

        if let Some(sram) = &mut self.sram {
            sram.reset();
        }

        // Clear block memory
        self.wram.fill(0);
        self.vram.fill(0);
        self.io_ports.fill(0);

        // Set various state
        self.access_oam = true;
        self.access_vram = true;
        self.rom_bank_offset = 0x4000;
        self.wram_bank_offset = 0x1000;
        self.vram_bank_offset = 0x0000;

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
        if sgb_flag {
            self.io_ports[38] = 0xf0;
        } else {
            self.io_ports[38] = 0xf1;
        }
    }
}

impl MemoryMap for MemBus {

    fn read_address(&self, address: usize) -> u8 {
        todo!()
    }

    fn write_address(&mut self, address: usize, byte: u8) {
        todo!()
    }
}
