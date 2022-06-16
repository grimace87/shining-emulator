
mod mem_bus;

pub use mem_bus::MemBus;

pub trait MemoryMap {
    fn read_address(&self, address: usize) -> u8;
    fn write_address(&mut self, address: usize, byte: u8);
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
}
