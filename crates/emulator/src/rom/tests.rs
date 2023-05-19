
use crate::Rom;
use files::local_rom_data;

#[test]
fn rom_loads() {
    let data = local_rom_data("test_rom.gb")
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
