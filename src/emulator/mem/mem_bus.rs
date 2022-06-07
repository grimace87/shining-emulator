
use crate::emulator::mem::MemoryMap;
use crate::emulator::rom::Rom;

pub struct MemBus {
    pub rom: Rom,
    pub wram: Vec<u8>,
    pub vram: Vec<u8>,
    pub io_ports: Vec<u8>,
    pub access_oam: bool,
    pub access_vram: bool,
    rom_bank_offset: usize,
    wram_bank_offset: usize,
    vram_bank_offset: usize
}

impl MemBus {

    pub fn new(rom: Rom) -> Self {
        Self {
            rom,
            wram: Vec::with_capacity(8 * 4096),
            vram: Vec::with_capacity(2 * 8192),
            io_ports: Vec::with_capacity(256),
            access_oam: false,
            access_vram: false,
            rom_bank_offset: 0,
            wram_bank_offset: 0,
            vram_bank_offset: 0
        }
    }

    pub fn reset(&mut self) {

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
