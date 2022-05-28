
pub struct MemBus {
    pub wram: Vec<u8>,
    pub vram: Vec<u8>,
    pub io_ports: Vec<u8>
}

impl MemBus {

    pub fn new() -> Self {
        Self {
            wram: Vec::with_capacity(8 * 4096),
            vram: Vec::with_capacity(2 * 8192),
            io_ports: Vec::with_capacity(256)
        }
    }

    pub fn reset(&mut self) {

    }
}
