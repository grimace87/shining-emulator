
use crate::emulator::{
    mem::MemoryMap,
    audio::AudioController
};

enum Mode {
    Running,
    Halted,
    Stopped
}

pub struct Cpu {
    pc: u32,
    sp: u32,
    a: u8,
    f: u8,
    b: u8,
    c: u8,
    d: u8,
    e: u8,
    h: u8,
    l: u8,
    mode: Mode,
    is_running: bool,
    is_paused: bool,
    ime: bool,
    divider_count: u32,
    current_key_dir: u8,
    current_key_but: u8
}

impl Cpu {

    pub fn new(sgb_flag: bool, cgb_flag: bool) -> Self {
        let mut cpu = Self {
            pc: 0,
            sp: 0,
            a: 0,
            f: 0,
            b: 0,
            c: 0,
            d: 0,
            e: 0,
            h: 0,
            l: 0,
            mode: Mode::Stopped,
            is_running: false,
            is_paused: false,
            ime: false,
            divider_count: 0,
            current_key_dir: 0x0f,
            current_key_but: 0x0f
        };

        // Return object ready to run
        cpu.reset(sgb_flag, cgb_flag);
        cpu
    }

    fn on_invalid_instruction(&mut self) {
        self.is_running = false;
    }

    pub fn reset(&mut self, sgb_flag: bool, cgb_flag: bool) {

        // Set various state
        self.pc = 0x0100;
        self.sp = 0xfffe;
        self.f = 0xb0;
        self.b = 0x00;
        self.c = 0x13;
        self.d = 0x00;
        self.e = 0xd8;
        self.h = 0x01;
        self.l = 0x4d;

        // Conditional state
        if cgb_flag {
            self.a = 0x11;
        } else if sgb_flag {
            self.a = 0x01;
        } else {
            self.a = 0x01;
        }
    }

    pub fn set_input_values(&mut self, key_dir: u8, key_but: u8) {
        self.current_key_dir = key_dir;
        self.current_key_but = key_but;
    }

    pub fn execute_accumulated_clocks<M: MemoryMap, A: AudioController>(
        &mut self,
        clocks: u64,
        mem_bus: &mut M,
        audio: &mut A
    ) {
        if !self.is_running || self.is_paused {
            return;
        }
    }
}
