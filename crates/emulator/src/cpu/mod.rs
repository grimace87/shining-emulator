
use crate::mem::MemBus;
use crate::audio::AudioController;
use crate::gpu::Gpu;

const GB_FREQ: u64 = 4194304;
const SGB_FREQ: u64 = 4295454;
const CGB_FREQ: u64 = 8400000;

#[derive(Eq, PartialEq, Copy, Clone)]
pub enum CpuType {
    Dmg,
    Sgb,
    Cgb
}

#[derive(Eq, PartialEq)]
enum Mode {
    Running,
    Halted,
    Stopped
}

pub struct Cpu {
    cpu_type: CpuType,
    pc: usize,
    sp: usize,
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
    clock_frequency: u64,
    ime: bool,
    divider_count: u32,
    timer_count: u32,
    timer_inc_time: u32,
    timer_running: bool,
    last_ly_compare: u32,
    current_key_dir: u8,
    current_key_but: u8,
    key_state_changed: bool
}

impl Cpu {

    pub fn new(cpu_type: CpuType) -> Self {
        let mut cpu = Self {
            cpu_type,
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
            clock_frequency: GB_FREQ,
            ime: false,
            divider_count: 0,
            timer_count: 0,
            timer_inc_time: 0,
            timer_running: false,
            last_ly_compare: 0,
            current_key_dir: 0x0f,
            current_key_but: 0x0f,
            key_state_changed: false
        };

        // Return object ready to run
        cpu.reset(cpu_type);
        cpu
    }

    fn on_invalid_instruction(&mut self) {
        self.is_running = false;
    }

    pub fn reset(&mut self, cpu_type: CpuType) {

        self.is_running = true;
        self.is_paused = false;
        self.ime = false;
        self.mode = Mode::Running;
        self.divider_count = 0;
        self.timer_count = 0;
        self.timer_inc_time = 1024;
        self.timer_running = false;
        self.last_ly_compare = 1;
        self.key_state_changed = false;

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

        // Hardware-dependent state
        match cpu_type {
            CpuType::Dmg => {
                self.a = 0x01;
                self.clock_frequency = GB_FREQ;
            },
            CpuType::Sgb => {
                self.a = 0x11;
                self.clock_frequency = SGB_FREQ;
            },
            CpuType::Cgb => {
                self.a = 0x11;
                self.clock_frequency = GB_FREQ;
            }
        }
    }

    pub fn get_clock_frequency(&self) -> u64 {
        self.clock_frequency
    }

    pub fn set_input_values(&mut self, key_dir: u8, key_but: u8) {
        if key_dir != self.current_key_dir || key_but != self.current_key_but {
            self.key_state_changed = true;
        }
        self.current_key_dir = key_dir;
        self.current_key_but = key_but;
    }

    pub fn emulate_clock_cycles<A: AudioController>(
        &mut self,
        clocks: i64,
        mem_bus: &mut MemBus,
        audio: &mut A,
        gpu: &mut Gpu
    ) -> i64 {
        if !self.is_running || self.is_paused {
            // Consume all clocks
            return clocks;
        }
        if self.key_state_changed {
            self.key_state_changed = false;

            // Adjust value in keypad register
            let upper_bits = mem_bus.read_address(0xff00) & 0x30;
            if upper_bits == 0x20 {
                mem_bus.perform_and(0xff00, 0xf0);
                mem_bus.perform_or(0xff00, self.current_key_dir);
            } else if upper_bits == 0x10 {
                mem_bus.perform_and(0xff00, 0xf0);
                mem_bus.perform_or(0xff00, self.current_key_but);
            }

            // Set interrupt request flag
            mem_bus.perform_or(0xff0f, 0x10);
        }

        let mut remaining_clocks = clocks;
        while remaining_clocks > 0 {
            let progress = self.emulate_next_step(mem_bus, audio, gpu) as i64;
            if progress <= 32 {
                let display_enabled = (mem_bus.read_address(0xff40) & 0x80) != 0;
                if display_enabled {
                    gpu.emulate_clock_cycles(progress, mem_bus);
                } else {
                    gpu.on_display_disabled(mem_bus);
                }
            }
            remaining_clocks -= progress;
        }
        clocks - remaining_clocks
    }

    /// Simulate one instruction, returning the number of clock cycles progressed.
    #[inline]
    fn emulate_next_step<A: AudioController>(
        &mut self,
        mem_bus: &mut MemBus,
        audio: &mut A,
        gpu: &mut Gpu
    ) -> u64 {
        let clocks_passed_by_instruction = self.perform_op(mem_bus, audio);
        self.pc &= 0xffff;

        // Check for interrupts
        let cpu_halted = self.mode == Mode::Halted;
        if self.ime || cpu_halted {
            self.process_interrupts(cpu_halted, mem_bus);
        }

        // While CPU is in stop mode, nothing much still runs
        if self.mode == Mode::Stopped {
            if let Some(gpu_clock_factor) = self.switch_running_speed(mem_bus) {
                self.mode = Mode::Running;
                gpu.clock_factor = gpu_clock_factor;
                return clocks_passed_by_instruction + 131072;
            }
            return clocks_passed_by_instruction;
        }

        // Permanent compare of LY and LYC
        let display_enabled = (mem_bus.read_address(0xff40) & 0x80) != 0;
        if display_enabled && mem_bus.read_address(0xff44) == mem_bus.read_address(0xff45) {

            // Set coincidence flag
            mem_bus.perform_or(0xff41, 0x04);

            // Request interrupt if his signal goes low to high
            if (mem_bus.read_address(0xff41) & 0x40) != 0 && self.last_ly_compare == 0 {
                mem_bus.perform_or(0xff0f, 0x02);
            }
            self.last_ly_compare = 1;
        } else {
            mem_bus.perform_and(0xff41, 0xfb);
            self.last_ly_compare = 0;
        }

        // Handling of timers
        self.divider_count += clocks_passed_by_instruction as u32;
        if self.divider_count >= 256 {
            self.divider_count -= 256;
            mem_bus.write_address(0xff04, mem_bus.read_address(0xff04) + 1);
        }
        if self.timer_running {
            self.timer_count += 1;
            if self.timer_count >= self.timer_inc_time {
                self.timer_count -= self.timer_inc_time;
                let new_register = mem_bus.read_address(0xff05) + 1;
                mem_bus.write_address(0xff05, new_register);
                if new_register == 0 {
                    mem_bus.write_address(0xff05, mem_bus.read_address(0xff06));
                    mem_bus.perform_or(0xff0f, 0x04);
                }
            }
        }

        // Handle audio
        audio.simulate(clocks_passed_by_instruction / (gpu.clock_factor as u64));

        // TODO - Serial functions

        clocks_passed_by_instruction
    }

    #[inline]
    fn process_interrupts(&mut self, cpu_halted: bool, mem_bus: &mut MemBus) {

        let triggered_interrupts = mem_bus.read_address(0xffff) &
            mem_bus.read_address(0xff0f) & 0x1f;
        if triggered_interrupts == 0 {
            return;
        }

        let mut to_address = self.pc;
        if (triggered_interrupts & 0x01) != 0 {
            // V-blank
            mem_bus.perform_and(0xff0f, 0x1e);
            to_address = 0x0040;
        } else if (triggered_interrupts & 0x02) != 0 {
            // LCD stat
            mem_bus.perform_and(0xff0f, 0x1d);
            to_address = 0x0048;
        } else if (triggered_interrupts & 0x04) != 0 {
            // Timer
            mem_bus.perform_and(0xff0f, 0x1b);
            to_address = 0x0050;
        } else if (triggered_interrupts & 0x08) != 0 {
            // Serial
            mem_bus.perform_and(0xff0f, 0x17);
            to_address = 0x0058;
        } else if (triggered_interrupts & 0x10) != 0 {
            // Joypad
            mem_bus.perform_and(0xff0f, 0x0f);
            to_address = 0x0060;
        }

        // Unless halted with IME unset, push PC onto stack and go to interrupt handler address
        if !cpu_halted || self.ime {
            self.sp -= 2;
            mem_bus.write_address_16(
                self.sp as usize, ((self.pc & 0xff) as u8, (self.pc >> 8) as u8));
            self.pc = to_address;
        }
        self.mode = Mode::Running;
        self.ime = false;
    }

    #[inline]
    fn switch_running_speed(&mut self, mem_bus: &mut MemBus) -> Option<u32> {
        let speed_change_requested = self.cpu_type == CpuType::Cgb && mem_bus.read_address(0xff4d) == 0x01;
        if speed_change_requested {
            // Speed change was requested in CGB mode
            mem_bus.perform_and(0xff4d, 0x80);
            let new_byte = mem_bus.read_address(0xff4d);
            if new_byte == 0x00 {
                mem_bus.write_address(0xff4d, 0x80);
                self.clock_frequency = CGB_FREQ;
                return Some(2);
            } else {
                mem_bus.write_address(0xff4d, 0x00);
                self.clock_frequency = GB_FREQ;
                return Some(1);
            }
        }
        None
    }

    #[inline]
    fn perform_op<A: AudioController>(
        &mut self,
        mem_bus: &mut MemBus,
        audio: &mut A
    ) -> u64 {
        todo!()
    }
}
