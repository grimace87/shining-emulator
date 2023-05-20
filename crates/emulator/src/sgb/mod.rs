
#[cfg(test)]
mod tests;

use crate::mem::MemBus;
use std::cmp::min;

const SGBCOM_PAL01: u32     = 0x00;
const SGBCOM_PAL23: u32     = 0x01;
const SGBCOM_PAL03: u32     = 0x02;
const SGBCOM_PAL12: u32     = 0x03;
const SGBCOM_ATTR_BLK: u32  = 0x04;
const SGBCOM_ATTR_LIN: u32  = 0x05;
const SGBCOM_ATTR_DIV: u32  = 0x06;
const SGBCOM_ATTR_CHR: u32  = 0x07;
const SGBCOM_SOUND: u32     = 0x08;
const SGBCOM_SOU_TRN: u32   = 0x09;
const SGBCOM_PAL_SET: u32   = 0x0a;
const SGBCOM_PAL_TRN: u32   = 0x0b;
const SGBCOM_ATRC_EN: u32   = 0x0c;
const SGBCOM_TEST_EN: u32   = 0x0d;
const SGBCOM_ICON_EN: u32   = 0x0e;
const SGBCOM_DATA_SEND: u32 = 0x0f;
const SGBCOM_DATA_TRN: u32  = 0x10;
const SGBCOM_MLT_REQ: u32   = 0x11;
const SGBCOM_JUMP: u32      = 0x12;
const SGBCOM_CHR_TRN: u32   = 0x13;
const SGBCOM_PCT_TRN: u32   = 0x14;
const SGBCOM_ATTR_TRN: u32  = 0x15;
const SGBCOM_ATTR_SET: u32  = 0x16;
const SGBCOM_MASK_EN: u32   = 0x17;
const SGBCOM_OBJ_TRN: u32   = 0x18;
const SGBCOM_PAL_PRI: u32   = 0x19;

pub struct Sgb {
    reading_command: bool,
    command_bytes: [[u32; 16]; 7],
    command_bits: [u32; 8],
    command: u32,
    read_command_bits: usize,
    read_command_bytes: usize,
    freeze_screen: bool,
    freeze_mode: u32,
    multi_enabled: bool,
    no_players: u32,
    no_packets_sent: usize,
    no_packets_to_send: usize,
    read_joypad_id: u32,
    mono_data: Vec<u32>,
    mapped_vram_for_trn_op: Vec<u8>,
    pub palettes_abgr: Vec<u32>,
    sys_palettes: Vec<u32>,
    chr_palettes: Vec<u32>
}

impl Sgb {

    pub fn new() -> Self {
        Self {
            reading_command: false,
            command_bytes: Default::default(),
            command_bits: Default::default(),
            command: 0,
            read_command_bits: 0,
            read_command_bytes: 0,
            freeze_screen: false,
            freeze_mode: 0,
            multi_enabled: false,
            no_players: 1,
            no_packets_sent: 0,
            no_packets_to_send: 0,
            read_joypad_id: 0,
            mono_data: vec![0; 160 * 152],
            mapped_vram_for_trn_op: vec![0; 4096],
            palettes_abgr: vec![0; 4 * 4],
            sys_palettes: vec![0; 512 * 4],
            chr_palettes: vec![0; 18 * 20]
        }
    }

    pub fn reset(&mut self) {
        self.reading_command = false;
        self.command = 0;
        self.read_command_bits = 0;
        self.read_command_bytes = 0;
        self.freeze_screen = false;
        self.freeze_mode = 0;
        self.multi_enabled = false;
        self.no_players = 1;
        self.no_packets_sent = 0;
        self.no_packets_to_send = 0;
        self.read_joypad_id = 0x0c;

        self.command_bytes.iter_mut().for_each(|packet| {
            packet.fill(0);
        });
        self.command_bits.fill(0);
        self.mono_data.fill(0);
        self.palettes_abgr.fill(0);
        self.sys_palettes.fill(0);
        self.chr_palettes.fill(0);
    }

    /// Signal an updated value on IO port 0x00.
    /// Packets are transferred by sending specific signals along this channel.
    /// Bits P15 and P14 are the only ones considered here.
    pub fn write_input_signal(&mut self, byte: u8, mem_bus: &mut MemBus) {

        let signal_bits = byte & 0x30;

        // P15 and P14 LOW signals RESET (start of packet)
        if !self.reading_command {
            if signal_bits == 0x00 {
                self.reading_command = true;
                self.read_command_bits = 0;
                self.read_command_bytes = 0;
                self.no_packets_sent = 0;
                self.no_packets_to_send = 1; // May be updated after first byte received
            }
            return;
        }

        // Command bits received by signalling one of two specific values
        if signal_bits == 0x20 {
            // Transfer a 0
            if self.read_command_bytes >= 16 {
                self.no_packets_sent += 1;
                self.read_command_bytes = 0;
                if self.no_packets_sent >= self.no_packets_to_send {
                    self.check_packets(mem_bus);
                    self.reading_command = false;
                }
                return;
            }
            self.command_bits[self.read_command_bits] = 0;
            self.read_command_bits += 1;
            if self.read_command_bits >= 8 {
                self.check_byte();
            }
            if self.no_packets_sent >= self.no_packets_to_send {
                self.check_packets(mem_bus);
                self.reading_command = false;
                self.no_packets_sent = 0;
                self.no_packets_to_send = 0;
            }
        } else if signal_bits == 0x10 {
            // Transfer a 1
            if self.read_command_bytes >= 16 {
                // Error in transmission - 1 at end of packet
                self.reading_command = false;
                return;
            }
            self.command_bits[self.read_command_bits] = 1;
            self.read_command_bits += 1;
            if self.read_command_bits >= 8 {
                self.check_byte();
            }
        }
    }

    fn check_byte(&mut self) {

        // TODO - Make this more graceful
        assert_eq!(self.read_command_bits, 8);

        // Copied bits sent into byte and increment byte counter
        let mut byte: u32 = self.command_bits[0] as u32;
        byte |= self.command_bits[1] << 1;
        byte |= self.command_bits[2] << 2;
        byte |= self.command_bits[3] << 3;
        byte |= self.command_bits[4] << 4;
        byte |= self.command_bits[5] << 5;
        byte |= self.command_bits[6] << 6;
        byte |= self.command_bits[7] << 7;
        self.command_bytes[self.no_packets_sent][self.read_command_bytes] = byte;
        self.read_command_bits = 0;
        self.read_command_bytes += 1;

        // If this is the first command byte, get its info
        if self.read_command_bytes == 1 && self.no_packets_sent == 0 {
            let byte = self.command_bytes[0][0];
            self.no_packets_to_send = byte as usize & 0x07;
            self.command = (byte >> 3) & 0x1f;
        }
    }

    fn check_packets(&mut self, mem_bus: &mut MemBus) {
        match self.command {
            SGBCOM_PAL01 => self.finalise_palette_load(0, 1),
            SGBCOM_PAL23 => self.finalise_palette_load(2, 3),
            SGBCOM_PAL03 => self.finalise_palette_load(0, 3),
            SGBCOM_PAL12 => self.finalise_palette_load(1, 2),
            SGBCOM_ATTR_BLK => self.finalise_attr_blk(),
            SGBCOM_ATTR_LIN => self.finalise_attr_lin(),
            SGBCOM_ATTR_DIV => self.finalise_attr_div(),
            SGBCOM_ATTR_CHR => self.finalise_attr_chr(),
            SGBCOM_SOUND => {},
            SGBCOM_PAL_SET => self.finalise_palette_set(),
            SGBCOM_PAL_TRN => self.finalise_palette_transfer(mem_bus),
            SGBCOM_ICON_EN => {},
            SGBCOM_DATA_SEND => {},
            SGBCOM_MLT_REQ => self.finalise_multi_request(),
            SGBCOM_CHR_TRN => {},
            SGBCOM_ATTR_TRN => {},
            SGBCOM_ATTR_SET => {},
            SGBCOM_PCT_TRN => {},
            SGBCOM_MASK_EN => self.finalise_mask_enable(),
            SGBCOM_PAL_PRI => {},
            _ => {}
        };
        self.command = 0;
    }

    fn finalise_palette_load(&mut self, first: usize, second: usize) {

        // Set first colour the same in all palettes
        let base = Self::remap_555_8888(
            self.command_bytes[0][1], self.command_bytes[0][2]);
        self.palettes_abgr[0] = base;
        self.palettes_abgr[4] = base;
        self.palettes_abgr[8] = base;
        self.palettes_abgr[12] = base;

        // Update requested palettes
        self.palettes_abgr[4 * first + 1] = Self::remap_555_8888(
            self.command_bytes[0][3], self.command_bytes[0][4]);
        self.palettes_abgr[4 * first + 2] = Self::remap_555_8888(
            self.command_bytes[0][5], self.command_bytes[0][6]);
        self.palettes_abgr[4 * first + 3] = Self::remap_555_8888(
            self.command_bytes[0][7], self.command_bytes[0][8]);
        self.palettes_abgr[4 * second + 1] = Self::remap_555_8888(
            self.command_bytes[0][9], self.command_bytes[0][10]);
        self.palettes_abgr[4 * second + 2] = Self::remap_555_8888(
            self.command_bytes[0][11], self.command_bytes[0][12]);
        self.palettes_abgr[4 * second + 3] = Self::remap_555_8888(
            self.command_bytes[0][13], self.command_bytes[0][14]);
    }

    #[inline]
    fn remap_555_8888(lo_byte: u32, hi_byte: u32) -> u32 {
        let double_byte: u32 = (hi_byte << 8) | lo_byte;
        ((double_byte & 0x001f) << 3) |
            ((double_byte & 0x03e0) << 6) |
            ((double_byte & 0x7c00) << 9) |
            0xff000000
    }

    fn finalise_attr_blk(&mut self) {

        // Get number of data groups
        let data_groups = self.command_bytes[0][1] & 0x1f;
        let mut packet_no = 0;
        let mut byte_no = 2;
        for _data_group in 0..data_groups {

            // Get control code and colour palette for this data group
            let area_flags = self.command_bytes[packet_no][byte_no] & 0x07;
            byte_no += 1;
            let palette_codes = self.command_bytes[packet_no][byte_no] & 0x3f;
            byte_no += 1;
            if byte_no >= 16 {
                byte_no = 0;
                packet_no += 1;
            }

            // Get top-left coordinates
            let x_left = self.command_bytes[packet_no][byte_no] as usize & 0x1f;
            byte_no += 1;
            let y_top = self.command_bytes[packet_no][byte_no] as usize & 0x1f;
            byte_no += 1;
            if byte_no >= 16 {
                byte_no = 0;
                packet_no += 1;
            }

            // Get bottom-right coordinates, clamping to avoid errors in this code
            let x_right = min(self.command_bytes[packet_no][byte_no] as usize & 0x1f, 19);
            byte_no += 1;
            let y_bottom = min(self.command_bytes[packet_no][byte_no] as usize & 0x1f, 17);
            byte_no += 1;
            if byte_no >= 16 {
                byte_no = 0;
                packet_no += 1;
            }

            // Check invalid conditions that may cause errors in this code
            if x_left > x_right || y_top > y_bottom {
                continue;
            }

            // Fill area outside block
            if area_flags > 3 {
                let p = (palette_codes & 0x30) >> 4;
                for y in 0..18 {
                    for x in 0..x_left {
                        self.chr_palettes[y * 20 + x] = p;
                    }
                    for x in (x_right + 1)..20 {
                        self.chr_palettes[y * 20 + x] = p;
                    }
                }
                for x in x_left..=x_right {
                    for y in 0..y_top {
                        self.chr_palettes[y * 20 + x] = p;
                    }
                    for y in (y_bottom + 1)..18 {
                        self.chr_palettes[y * 20 + x] = p;
                    }
                }
            }

            // Fill area inside block
            if (area_flags & 0x01) > 0 {
                let p = palette_codes & 0x03;
                for y in (y_top + 1)..y_bottom {
                    for x in (x_left + 1)..x_right {
                        self.chr_palettes[y * 20 + x] = p;
                    }
                }
            }

            // Fill block border
            if area_flags > 0 && area_flags != 5 {
                let p = match area_flags {
                    1 => palette_codes & 0x03,
                    4 => (palette_codes & 0x30) >> 4,
                    _ => (palette_codes & 0x0c) >> 2
                };
                for y in y_top..=y_bottom {
                    self.chr_palettes[y * 20 + x_left] = p;
                    self.chr_palettes[y * 20 + x_right] = p;
                }
                for x in x_left..=x_right {
                    self.chr_palettes[y_top * 20 + x] = p;
                    self.chr_palettes[y_bottom * 20 + x] = p;
                }
            }
        }
    }

    fn finalise_attr_lin(&mut self) {
        self.command = 0;
    }

    fn finalise_attr_div(&mut self) {
        let is_divider_horizontal = (self.command_bytes[0][1] & 0x40) != 0;
        let palette_numbers = self.command_bytes[0][1] & 0x3f;
        let coordinate = (self.command_bytes[0][2] & 0x1f) as usize;
        if is_divider_horizontal {
            let divider_y = min(coordinate, 17);

            // Draw before the line
            let palette_no = (palette_numbers & 0x0c) >> 2;
            for y in 0..divider_y {
                for x in 0..20 {
                    self.chr_palettes[y * 20 + x] = palette_no;
                }
            }

            // Draw on the line
            let palette_no = (palette_numbers & 0x30) >> 4;
            for x in 0..20 {
                self.chr_palettes[divider_y * 20 + x] = palette_no;
            }

            // Draw after the line
            let palette_no = palette_numbers & 0x03;
            for y in (divider_y + 1)..18 {
                for x in 0..20 {
                    self.chr_palettes[y * 20 + x] = palette_no;
                }
            }
        } else {
            let divider_x = min(coordinate, 19);

            // Draw before the line
            let palette_no = (palette_numbers & 0x0c) >> 2;
            for y in 0..18 {
                for x in 0..divider_x {
                    self.chr_palettes[y * 20 + x] = palette_no;
                }
            }

            // Draw on the line
            let palette_no = (palette_numbers & 0x30) >> 4;
            for y in 0..18 {
                self.chr_palettes[y * 20 + divider_x] = palette_no;
            }

            // Draw after the line
            let palette_no = palette_numbers & 0x03;
            for y in 0..18 {
                for x in (divider_x + 1)..20 {
                    self.chr_palettes[y * 20 + x] = palette_no;
                }
            }
        }
    }

    fn finalise_attr_chr(&mut self) {
        let mut x = min((self.command_bytes[0][1] & 0x1f) as usize, 19);
        let mut y = min((self.command_bytes[0][2] & 0x1f) as usize, 17);
        let data_set_count =
            (256 * (self.command_bytes[0][4] & 0x01) + self.command_bytes[0][3]) as usize;
        let scan_horizontal = (self.command_bytes[0][5] & 0x01) == 0;
        let mut palette_index: usize = 0;
        let mut packet_no = 0;
        let mut byte_in_packet = 6;

        for _set in 0..data_set_count {

            // Write next colour
            let current_byte = self.command_bytes[packet_no][byte_in_packet];
            self.chr_palettes[y * 20 + x] = match palette_index {
                0 => (current_byte & 0xc0) >> 6,
                1 => (current_byte & 0x30) >> 4,
                2 => (current_byte & 0x0c) >> 2,
                _ => current_byte & 0x03
            };

            // Move to next character
            if scan_horizontal {
                x += 1;
                if x >= 20 {
                    x = 0;
                    y += 1;
                    if y >= 18 {
                        return;
                    }
                }
            } else {
                y += 1;
                if y >= 18 {
                    y = 0;
                    x += 1;
                    if x >= 20 {
                        return;
                    }
                }
            }

            // Move to next palette number
            palette_index += 1;
            if palette_index > 3 {
                palette_index = 0;
                byte_in_packet += 1;
                if byte_in_packet > 15 {
                    byte_in_packet = 0;
                    packet_no += 1;
                    if packet_no >= self.no_packets_sent {
                        return;
                    }
                }
            }
        }
    }

    fn finalise_palette_set(&mut self) {
        for palette_no in 0..4 {
            let offset = palette_no * 2 + 1;
            let source_palette_no =
                (256 * (self.command_bytes[0][offset + 1] & 0x01) + self.command_bytes[0][offset])
                    as usize;
            for colour_no in 0..4 {
                self.palettes_abgr[palette_no * 4 + colour_no] =
                    self.sys_palettes[source_palette_no * 4 + colour_no];
            }
        }
        if (self.command_bytes[0][9] & 0x40) != 0 {
            self.freeze_mode = 0;
            self.freeze_screen = false;
        }
    }

    fn finalise_palette_transfer(&mut self, mem_bus: &mut MemBus) {
        let display_flags = mem_bus.read_address(0xff40);
        if (display_flags & 0x80) != 0 {
            self.map_vram_for_transfer_op(mem_bus);
            let mut source_index: usize = 0;
            for palette_no in 0..512 {
                for colour_no in 0..4 {
                    self.sys_palettes[palette_no * 4 + colour_no] = Self::remap_555_8888(
                        self.mapped_vram_for_trn_op[source_index] as u32,
                        self.mapped_vram_for_trn_op[source_index + 1] as u32
                    )
                }
            }
        }
    }

    fn finalise_multi_request(&mut self) {
        self.multi_enabled = self.command_bytes[0][1] & 0x01 != 0;
        self.no_players = self.command_bytes[0][1] + 0x01;
        self.read_joypad_id = 0x0f;
    }

    fn finalise_mask_enable(&mut self) {
        self.freeze_screen = match self.command_bytes[0][1] {
            0 => false,
            1 => true,
            2 => true,
            3 => true,
            _ => false
        };
        self.freeze_mode = self.command_bytes[0][1];
    }

    // Copy data from the VRAM signal
    // Assumes a certain display configuration and does not account for variances
    // This includes display enable, background not scrolled, window and sprites not on-screen,
    // and the BGP palette register has a certain value (possibly 0xe4)
    fn map_vram_for_transfer_op(&mut self, mem_bus: &mut MemBus) {

        let display_flags = mem_bus.read_address(0xff40);
        let use_low_chr_offset = (display_flags & 0x10) != 0;
        let (chars_start, char_code_inverter) = match use_low_chr_offset {
            true => (0 as usize, 0 as usize),
            false => (0x0800 as usize, 0x0080 as usize)
        };

        let use_high_map_offset = (display_flags & 0x08) != 0;
        let map_start = match use_high_map_offset {
            true => 0x1c00 as usize,
            false => 0x1800 as usize
        };

        for chr_y in 0..18 {
            for chr_x in 0..20 {
                // Copy 16 bytes of the character tile in this location
                let map_index = chr_y * 32 + chr_x;
                let tile_no_byte = mem_bus.read_address(0x8000 + map_start + map_index);
                let zero_based_tile_no = (tile_no_byte as usize) ^ char_code_inverter;
                let mut chars_data_start_index = chars_start + zero_based_tile_no * 16;
                for byte_no in 0..16 {
                    self.mapped_vram_for_trn_op[16 * (chr_y * 20 + chr_x) + byte_no] =
                        mem_bus.read_address(0x8000 + chars_data_start_index);
                    chars_data_start_index += 1;
                }
            }
        }
    }

    pub fn colourise_frame(&self, image_data: &mut [u32]) {
        for chrx in 0..20 {
            for chry in 0..18 {
                let palette_number = self.chr_palettes[chry * 20 + chrx];
                let palette_index = palette_number as usize * 4;
                let x_max = 8 * chrx + 8;
                let y_max = 8 * chry + 8;
                for px in (8 * chrx)..x_max {
                    for py in (8 * chry)..y_max {
                        image_data[py * 160 + px] = self.palettes_abgr[palette_index + self.mono_data[160 * py + px] as usize];
                    }
                }
            }
        }
    }
}
