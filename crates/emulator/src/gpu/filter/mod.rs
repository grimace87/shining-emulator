/*
 * XBR filter: from the FFmpeg project
 *
 * Copyright (c) 2011, 2012 Hyllian/Jararaca <sergiogdb@gmail.com>
 * Copyright (c) 2014 Arwa Arif <arwaarif1994@gmail.com>
 * Copyright (c) 2015 Treeki <treeki@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/*
 * Modified by Thomas Reichert
 * Removed DLL-specific declspec modifiers to allow building as a static lib.
 * Removed hqx functionality.
 * Unravelled some macros.
 * Converted to Rust.
 * Converted macros to functions, to favour safety over performance.
 *
 * Algorithm explained:
 * https://forums.libretro.com/t/xbr-algorithm-tutorial/123
 */

mod xbr_params;

pub use xbr_params::XbrParams;

#[cfg(test)]
mod tests {
    use super::XbrParams;

    fn make_params(scale: usize) -> XbrParams {
        let input_size: usize = 8;
        XbrParams::new(
            scale,
            input_size,
            input_size)
    }

    #[test]
    fn can_filter_2x() {
        let mut params = make_params(2);
        params.xbr_filter_xbr2x();
    }

    #[test]
    fn can_filter_3x() {
        let mut params = make_params(3);
        params.xbr_filter_xbr3x();
    }

    #[test]
    fn can_filter_4x() {
        let mut params = make_params(4);
        params.xbr_filter_xbr4x();
    }
}
