
mod cpu;
pub mod rom;
mod mem;
mod sgb;
mod input;

use cpu::Cpu;
use mem::MemBus;
use rom::Rom;
use sgb::Sgb;
use input::Input;

use std::cell::RefCell;

const GB_FREQ: u64 = 4194304;

pub struct Emulator {
    cpu: Cpu,
    mem_bus: RefCell<MemBus>,
    sgb: Sgb,
    input: Input,
    wram: Vec<u8>,
    vram: Vec<u8>,
    io_ports: Vec<u8>,
    cpu_clock_frequency: u64,
    divider_count: u32,
    accumulated_clocks: u64,
    clock_multiply: i64,
    clock_divide: i64,
    current_clock_multiplier_combo: i32
}

impl Emulator {

    pub fn load_rom(rom: Rom) -> Self {
        let mut emulator = Self {
            cpu: Cpu::new(rom.sgb_flag, rom.cgb_flag),
            mem_bus: RefCell::new(MemBus::new(rom)),
            sgb: Sgb::new(),
            input: Input::new(),
            wram: Vec::with_capacity(8 * 4096),
            vram: Vec::with_capacity(2 * 8192),
            io_ports: Vec::with_capacity(256),
            cpu_clock_frequency: GB_FREQ,
            divider_count: 1,
            accumulated_clocks: 0,
            clock_multiply: 1,
            clock_divide: 1,
            current_clock_multiplier_combo: 10
        };

        emulator
    }

    pub fn reset(&mut self) {

        let (sgb_flag, cgb_flag) = {
            let rom = &self.mem_bus.borrow().rom;
            (rom.sgb_flag, rom.cgb_flag)
        };

        // Reset component states
        self.cpu.reset(sgb_flag, cgb_flag);
        self.sgb.reset();
        self.input.reset();

        // Reset clock multipliers
        self.current_clock_multiplier_combo = 10;
        self.clock_multiply = 1;
        self.clock_divide = 1;

        // Initialise control variables

    }
}
