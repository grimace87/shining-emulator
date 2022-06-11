
mod mem_bus;

pub use mem_bus::MemBus;

pub trait MemoryMap {
    fn read_address(&self, address: usize) -> u8;
    fn write_address(&mut self, address: usize, byte: u8);
    fn read_address_16(&self, address: usize) -> (u8, u8);
    fn write_address_16(&mut self, address: usize, bytes: (u8, u8));
    fn set_access_vram(&mut self, accessible: bool);
    fn set_access_oam(&mut self, accessible: bool);
    fn perform_and(&mut self, address: usize, byte: u8);
    fn perform_or(&mut self, address: usize, byte: u8);
}

pub struct SimpleMemoryMap {
    store: Vec<u8>
}

impl SimpleMemoryMap {

    pub fn new() -> Self {
        Self { store: vec![0; 16 * 8192] }
    }
}

impl MemoryMap for SimpleMemoryMap {

    fn read_address(&self, address: usize) -> u8 {
        self.store[address & 0xffff]
    }

    fn write_address(&mut self, address: usize, byte: u8) {
        self.store[address] = byte;
    }

    fn read_address_16(&self, address: usize) -> (u8, u8) {
        (self.store[address + 1], self.store[address])
    }

    fn write_address_16(&mut self, address: usize, bytes: (u8, u8)) {
        self.store[address] = bytes.1;
        self.store[address + 1] = bytes.0;
    }

    fn set_access_vram(&mut self, accessible: bool) {}

    fn set_access_oam(&mut self, accessible: bool) {}

    fn perform_and(&mut self, address: usize, byte: u8) {
        self.store[address] &= byte;
    }

    fn perform_or(&mut self, address: usize, byte: u8) {
        self.store[address] |= byte;
    }
}
