use std::path::PathBuf;

pub fn local_rom_data(rom_name: &str) -> Result<Vec<u8>, String> {
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    path.push("roms");
    path.push(rom_name);
    std::fs::read(path)
        .map_err(|e| format!("{:?}", e))
}
