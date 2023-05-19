
mod mem_bus;

pub use mem_bus::MemBus;

pub trait MemoryMap {
    fn read_address(&self, address: usize) -> u8;
    fn write_address(&mut self, address: usize, byte: u8);
    fn read_address_16(&self, address: usize) -> (u8, u8);
    fn write_address_16(&mut self, address: usize, bytes: (u8, u8));
    fn set_vram_protection(&mut self, protected: bool);
    fn set_oam_protection(&mut self, protected: bool);
    fn perform_and(&mut self, address: usize, byte: u8);
    fn perform_or(&mut self, address: usize, byte: u8);
}

pub struct SimpleMemoryMap {
    store: Vec<u8>,
    vram_protected: bool,
    oam_protected: bool
}

impl SimpleMemoryMap {

    pub fn new() -> Self {
        Self { store: vec![0; 16 * 8192], vram_protected: false, oam_protected: false }
    }
}

impl MemoryMap for SimpleMemoryMap {

    fn read_address(&self, address: usize) -> u8 {
        if address >= 0x8000 && address < 0xa000 && self.vram_protected {
            return 0xff;
        }
        if address >= 0xfe00 && address < 0xfea0 && self.oam_protected {
            return 0xff;
        }
        self.store[address & 0xffff]
    }

    fn write_address(&mut self, address: usize, byte: u8) {
        if address >= 0x8000 && address < 0xa000 && self.vram_protected {
            return;
        }
        if address >= 0xfe00 && address < 0xfea0 && self.oam_protected {
            return;
        }
        self.store[address] = byte;
    }

    fn read_address_16(&self, address: usize) -> (u8, u8) {
        (self.store[address + 1], self.store[address])
    }

    fn write_address_16(&mut self, address: usize, bytes: (u8, u8)) {
        self.store[address] = bytes.1;
        self.store[address + 1] = bytes.0;
    }

    fn set_vram_protection(&mut self, protected: bool) {
        self.vram_protected = protected;
    }

    fn set_oam_protection(&mut self, protected: bool) {
        self.oam_protected = protected;
    }

    fn perform_and(&mut self, address: usize, byte: u8) {
        self.store[address] &= byte;
    }

    fn perform_or(&mut self, address: usize, byte: u8) {
        self.store[address] |= byte;
    }
}
