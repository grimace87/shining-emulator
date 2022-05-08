
mod emulator;
mod files;

const ROM_NAME: &str = "test_rom.gb";

fn main() {
    println!("Hello, world!");
}

#[cfg(test)]
mod test {
    use crate::emulator::rom::Rom;
    use crate::files::local_rom_data;
    use crate::ROM_NAME;

    #[test]
    fn rom_loads() {
        let data = local_rom_data(ROM_NAME)
            .unwrap();
        let rom = Rom::new_from_data(data);
        assert!(rom.valid);
        assert!(!rom.cgb_flag);
        assert!(!rom.sgb_flag);
        assert!(rom.has_sram);
        assert!(rom.has_battery);
        assert!(!rom.has_timer);
        assert!(!rom.has_rumble);
        assert_eq!(rom.size_bytes, 131072);
        assert_eq!(rom.sram_size_bytes, 512);
    }
}
