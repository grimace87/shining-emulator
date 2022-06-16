
use crate::rom::Rom;
use crate::external_ram::ExternalRam;

use dirs::home_dir;
use std::fs::File;
use std::io::{BufWriter, Read, Seek, Write, Error, ErrorKind};
use std::path::PathBuf;

const HOME_SAVE_DIR: &str = ".shining-emulator";
const SAVE_FILE_EXTENSION: &str = "gsv";

pub struct Sram {
    file_backing: Option<BufWriter<File>>,
    data: Vec<u8>,
    has_timer: bool,
    timer_data: [u8; 5],
    timer_mode: u32,
    timer_latch: u32,
    bank_offset: usize,
    size_enum: u8,
    size_bytes: usize,
    bank_select_mask: u8,
    enable_flag: bool
}

impl Sram {

    pub fn new(rom: &Rom) -> Result<Self, Error> {

        let expected_size = rom.sram_size_bytes + if rom.has_timer { 5 } else { 0 };
        let mut data_buffer: Vec<u8> = vec![0; expected_size];

        let file_backing: Option<BufWriter<File>> = match rom.has_battery {
            true => {

                // Attempt to load current data and copy into the above buffer
                if let Ok(current_data) = Self::get_file_contents(&rom.name) {
                    if current_data.len() > data_buffer.len() {
                        data_buffer.copy_from_slice(&current_data[0..expected_size]);
                    } else {
                        data_buffer.copy_from_slice(&current_data.as_slice());
                    }
                }

                // Attempt to open file for writing; if it exists it will be truncated.
                // Write the data read above (or the zeroed buffer) to preserve file contents
                let mut file = Self::open_output_file(&rom.name)?;
                file.write(&data_buffer)?;
                file.rewind()?;
                Some(file)
            },
            false => None
        };

        let mut sram = Self {
            file_backing,
            data: data_buffer,
            has_timer: rom.has_timer,
            timer_data: [0, 0, 0, 0, 0],
            timer_mode: 0,
            timer_latch: 0,
            bank_offset: 0,
            size_enum: 0,
            size_bytes: rom.sram_size_bytes,
            bank_select_mask: 0,
            enable_flag: false
        };

        // Return object ready to run
        sram.reset();
        Ok(sram)
    }

    pub fn reset(&mut self) {
        self.data.fill(0);
        self.timer_data.fill(0);
        self.timer_mode = 0;
        self.timer_latch = 0;
        self.bank_offset = 0;
        self.bank_select_mask = 0;
        self.enable_flag = false;
    }

    fn get_sram_file(rom_name: &str) -> Result<PathBuf, Error> {
        let mut home_dir = match home_dir() {
            Some(dir) => dir,
            None => return Err(Error::new(ErrorKind::Other, "No home directory"))
        };
        home_dir.push(HOME_SAVE_DIR);
        home_dir.push(rom_name);
        home_dir.set_extension(SAVE_FILE_EXTENSION);
        Ok(home_dir)
    }

    fn get_file_contents(rom_name: &str) -> Result<Vec<u8>, Error> {
        let path = Self::get_sram_file(rom_name)?;
        let mut file = File::open(&path)?;
        let metadata = std::fs::metadata(&path)?;
        let mut buffer = vec![0; metadata.len() as usize];
        file.read(&mut buffer)?;
        Ok(buffer)
    }

    fn open_output_file(rom_name: &str) -> Result<BufWriter<File>, Error> {
        let path = Self::get_sram_file(rom_name)?;
        let file = File::create(path)?;
        let writer = BufWriter::new(file);
        Ok(writer)
    }

    pub fn write_timer_data(&mut self, timer_mode: usize, byte: u8) {
        let address = self.size_bytes + timer_mode;
        self.write_through(address, byte);
    }

    fn write_through(&mut self, address: usize, byte: u8) {
        self.data[address] = byte;
        if let Some(file) = &mut self.file_backing {
            if let Err(e) = file.seek(std::io::SeekFrom::Start(address as u64)) {
                println!("Error writing to backing file: {:?}", e);
                self.file_backing = None;
                return;
            }
            let buffer = [byte];
            file.write(&buffer).expect("Seek worked in file but not write");
        }
    }
}

impl ExternalRam for Sram {

    fn read_byte(&self, address: usize) -> u8 {
        self.data[self.bank_offset + (address & 0x1fff)]
    }

    fn write_byte(&mut self, address: usize, byte: u8) {
        let store_address = (address & 0x1fff) % self.size_bytes + self.bank_offset;
        self.write_through(store_address, byte);
    }
}
