
use crate::mem::MemBus;
use crate::rom::Rom;
use crate::sgb::Sgb;

fn send_byte(sgb: &mut Sgb, byte: u8, mem_bus: &mut MemBus) {
    let bits_as_bytes: [u8; 8] = [
        (byte & 0x01),
        (byte & 0x02 ) >> 1,
        (byte & 0x04 ) >> 2,
        (byte & 0x08 ) >> 3,
        (byte & 0x10 ) >> 4,
        (byte & 0x20 ) >> 5,
        (byte & 0x40 ) >> 6,
        (byte & 0x80 ) >> 7
    ];
    for bit_as_byte in bits_as_bytes {
        let signal_byte: u8 = if bit_as_byte == 0 { 0x20 } else { 0x10 };
        sgb.write_input_signal(0x30, mem_bus);
        sgb.write_input_signal(signal_byte, mem_bus);
        sgb.write_input_signal(0x30, mem_bus);
    }
}

fn send_packet(sgb: &mut Sgb, mem_bus: &mut MemBus, bytes: [u8; 16]) {

    // Reset signal
    sgb.write_input_signal(0x30, mem_bus);
    sgb.write_input_signal(0x00, mem_bus);
    sgb.write_input_signal(0x30, mem_bus);

    // Send packet command/parameter data
    for byte in bytes {
        send_byte(sgb, byte, mem_bus);
    }

    // Stop '0' bit
    sgb.write_input_signal(0x30, mem_bus);
    sgb.write_input_signal(0x20, mem_bus);
    sgb.write_input_signal(0x30, mem_bus);
}

#[test]
fn pal_commands_update_palettes() {

    let mut mem_bus = MemBus::new(Rom::default());
    let mut sgb = Sgb::new();
    sgb.reset();

    // Test PAL01
    let expected_palette_0: [u32; 4] = [0xff38d8e0, 0xffc04018, 0xff38d8e0, 0xffc04018];
    let expected_palette_1: [u32; 4] = [0xff38d8e0, 0xff38d8e0, 0xffc04018, 0xff38d8e0];
    send_packet(&mut sgb, &mut mem_bus, [
        0x01, // Command + length
        0x7c, 0x1f, 0x03, 0x61, 0x7c, 0x1f, 0x03, 0x61, // Palette 0 colours 0-3
        0x7c, 0x1f, 0x03, 0x61, 0x7c, 0x1f, // Palette 1 colours 1-3
        0x00
    ]);
    assert_eq!(&sgb.palettes_abgr[0..4], expected_palette_0);
    assert_eq!(&sgb.palettes_abgr[4..8], expected_palette_1);

    // Test PAL23
    let expected_palette_2: [u32; 4] = [0xff38d8e0, 0xffc04018, 0xff38d8e0, 0xffc04018];
    let expected_palette_3: [u32; 4] = [0xff38d8e0, 0xff38d8e0, 0xffc04018, 0xff38d8e0];
    send_packet(&mut sgb, &mut mem_bus, [
        0x09, // Command + length
        0x7c, 0x1f, 0x03, 0x61, 0x7c, 0x1f, 0x03, 0x61, // Palette 2 colours 0-3
        0x7c, 0x1f, 0x03, 0x61, 0x7c, 0x1f, // Palette 3 colours 1-3
        0x00
    ]);
    assert_eq!(&sgb.palettes_abgr[0..4], expected_palette_2);
    assert_eq!(&sgb.palettes_abgr[4..8], expected_palette_3);

    // Test PAL03
    let expected_palette_0: [u32; 4] = [0xff38d8e0, 0xffc04018, 0xff38d8e0, 0xffc04018];
    let expected_palette_3: [u32; 4] = [0xff38d8e0, 0xff38d8e0, 0xffc04018, 0xff38d8e0];
    send_packet(&mut sgb, &mut mem_bus, [
        0x11, // Command + length
        0x7c, 0x1f, 0x03, 0x61, 0x7c, 0x1f, 0x03, 0x61, // Palette 0 colours 0-3
        0x7c, 0x1f, 0x03, 0x61, 0x7c, 0x1f, // Palette 3 colours 1-3
        0x00
    ]);
    assert_eq!(&sgb.palettes_abgr[0..4], expected_palette_0);
    assert_eq!(&sgb.palettes_abgr[4..8], expected_palette_1);

    // Test PAL12
    let expected_palette_1: [u32; 4] = [0xff38d8e0, 0xffc04018, 0xff38d8e0, 0xffc04018];
    let expected_palette_2: [u32; 4] = [0xff38d8e0, 0xff38d8e0, 0xffc04018, 0xff38d8e0];
    send_packet(&mut sgb, &mut mem_bus, [
        0x19, // Command + length
        0x7c, 0x1f, 0x03, 0x61, 0x7c, 0x1f, 0x03, 0x61, // Palette 1 colours 0-3
        0x7c, 0x1f, 0x03, 0x61, 0x7c, 0x1f, // Palette 2 colours 1-3
        0x00
    ]);
    assert_eq!(&sgb.palettes_abgr[0..4], expected_palette_0);
    assert_eq!(&sgb.palettes_abgr[4..8], expected_palette_1);
}

#[test]
fn attr_commands_colour_areas() {

    let mut mem_bus = MemBus::new(Rom::default());
    let mut sgb = Sgb::new();
    sgb.reset();

    // Test ATTR_BLK with one data set (which fits in one 16-byte packet)
    // Assign palette 1 outside, palette 2 on the border, palette 3 inside
    // Use border bounds of (2,2) - (6,6)
    send_packet(&mut sgb, &mut mem_bus, [
        0x21, 0x01, // Command + length, number of data sets
        0x07, 0x1b, 0x02, 0x02, 0x06, 0x06, // Data set 1
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    ]);
    let expected_chr: [u32; 360] = [
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 2, 3, 3, 3, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 2, 3, 3, 3, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 2, 3, 3, 3, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
    ];
    assert_eq!(sgb.chr_palettes.as_slice(), expected_chr);

    // Test ATTR_BLK with three data sets, requiring 2 16-byte packets
    // For the first set, assign only inside, which colours the border as well
    // It will assign palette 0 within (0,2) - (2,8)
    // The second and third will both assign palette 0 to a border and 3 within; the bounds
    // will be (8,2) - (10,4) and (8,6) - (10,8)
    send_packet(&mut sgb, &mut mem_bus, [
        0x22, 0x03, // Command + length, number of data sets
        0x01, 0x00, 0x00, 0x02, 0x02, 0x08, // Data set 1
        0x03, 0x03, 0x08, 0x02, 0x0a, 0x04, // Data set 2
        0x03, 0x03 // Start of data set 3
    ]);
    send_packet(&mut sgb, &mut mem_bus, [
        0x08, 0x06, 0x0a, 0x08, // Data set 3 continued
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    ]);
    let expected_chr: [u32; 360] = [
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        0, 0, 0, 2, 2, 2, 2, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        0, 0, 0, 3, 3, 3, 2, 1, 0, 3, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        0, 0, 0, 3, 3, 3, 2, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        0, 0, 0, 3, 3, 3, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        0, 0, 0, 2, 2, 2, 2, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        0, 0, 0, 1, 1, 1, 1, 1, 0, 3, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
    ];
    assert_eq!(sgb.chr_palettes.as_slice(), expected_chr);

    // Test ATTR_DIV, assigning palette 0 above a horizontal line at y = 14, palette 1 along
    // the line, and palette 2 below
    send_packet(&mut sgb, &mut mem_bus, [
        0x31, // Command + length
        0x52, // Direction bit and pallete numbers
        0x0e, // Coordinate of dividing line
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    ]);
    let expected_chr: [u32; 360] = [
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2
    ];
    assert_eq!(sgb.chr_palettes.as_slice(), expected_chr);

    // Test ATTR_CHR by sending 3 packets containing <x NUMBER OF PALETTE INDICES> palette indices.
    // These indices will start at (8,8), scanning vertically, and repeat a pattern of
    // palette indices 0, 1, 2, 3.
    send_packet(&mut sgb, &mut mem_bus, [
        0x3b, // Command + length
        0x08, 0x08, 0x64, 0x00, 0x01, // start X, start Y, number of data sets, scan direction
        0x1b, 0x1b,
        0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b
    ]);
    send_packet(&mut sgb, &mut mem_bus, [
        0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b,
        0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b
    ]);
    send_packet(&mut sgb, &mut mem_bus, [
        0x1b, 0x1b, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    ]);
    let expected_chr: [u32; 360] = [
        0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2, 0, 2, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 1, 3, 1, 3, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 3, 1, 3, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2, 0, 2, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 1, 3, 1, 3, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 3, 1, 3, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2, 0, 2, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 3, 1, 3, 1, 3, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2, 0, 2, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 3, 1, 3, 1, 3, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2, 0, 2, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 3, 1, 3, 1, 3, 0, 0, 0, 0, 0, 0,
        1, 1, 1, 1, 1, 1, 1, 1, 2, 0, 2, 0, 2, 0, 1, 1, 1, 1, 1, 1,
        2, 2, 2, 2, 2, 2, 2, 2, 3, 1, 3, 1, 3, 1, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 0, 2, 0, 2, 0, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 1, 3, 1, 3, 1, 3, 2, 2, 2, 2, 2, 2
    ];
    assert_eq!(sgb.chr_palettes.as_slice(), expected_chr);
}
