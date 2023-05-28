
mod sram;

pub use sram::Sram;

use std::num::Wrapping;

pub trait ExternalRam {
    fn read_byte(&self, address: Wrapping<usize>) -> Wrapping<u8>;
    fn write_byte(&mut self, address: Wrapping<usize>, byte: Wrapping<u8>);
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

    fn read_byte(&self, address: Wrapping<usize>) -> Wrapping<u8> {
        Wrapping(self.store[address.0])
    }

    fn write_byte(&mut self, address: Wrapping<usize>, byte: Wrapping<u8>) {
        self.store[address.0] = byte.0;
    }
}
