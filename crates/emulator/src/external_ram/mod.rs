
mod sram;

pub use sram::Sram;

pub trait ExternalRam {
    fn read_byte(&self, address: usize) -> u8;
    fn write_byte(&mut self, address: usize, byte: u8);
}

pub struct SimpleExternalRam {
    store: Vec<u8>
}

impl SimpleExternalRam {

    pub fn new() -> Self {
        Self { store: vec![0; 16 * 8192] }
    }
}

impl ExternalRam for SimpleExternalRam {

    fn read_byte(&self, address: usize) -> u8 {
        self.store[address]
    }

    fn write_byte(&mut self, address: usize, byte: u8) {
        self.store[address] = byte;
    }
}
