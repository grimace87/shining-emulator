
use std::cell::RefCell;

const PART_MASK: usize = 0x00ff00ff;
const Y_MASK: usize = 0x00ff0000;
const U_MASK: usize = 0x0000ff00;
const V_MASK: usize = 0x000000ff;

const CONST_ZERO: usize = 0;
const CONST_ONE: usize = 1;
const CONST_TWO: usize = 2;
const CONST_THREE: usize = 3;

pub struct XbrParams {
    rgb_to_yuv: Vec<u32>,
    pub input_width: usize,
    pub input_height: usize,
    pub input: RefCell<Vec<u32>>,
    pub input_pitch: usize,
    pub output: RefCell<Vec<u32>>,
    pub output_pitch: usize
}

impl XbrParams {

    pub fn new(
        scale_factor: usize,
        input_width: usize,
        input_height: usize
    ) -> Self {
        let input_width_bytes = input_width * std::mem::size_of::<u32>();
        let input_buffer: Vec<u32> = vec![0; input_width * input_height];
        let output_buffer: Vec<u32> =
            vec![0; input_width * scale_factor * input_height * scale_factor];
        Self {
            rgb_to_yuv: Self::generate_rgb_to_yuv_conversion(),
            input_width,
            input_height,
            input: RefCell::new(input_buffer),
            input_pitch: input_width_bytes,
            output: RefCell::new(output_buffer),
            output_pitch: input_width_bytes * scale_factor
        }
    }

    fn generate_rgb_to_yuv_conversion() -> Vec<u32> {
        let mut rgb_to_yuv: Vec<u32> = vec![0; 1 << 24];

        for bg in (-255 as isize)..256 {
            for rg in (-255 as isize)..256 {
                let u = (-169 * rg + 500 * bg) / 1000 + 128;
                let v = (500 * rg - 81 * bg) / 1000 + 128;
                let start_g = (-bg).max((-rg).max(0));
                let end_g = (255 - bg).min((255 - rg).min(255));
                let mut y = (299 * rg + 1000 * start_g + 114 * bg) / 1000;
                let mut c = bg + (rg << 16) + 0x010101 * start_g;
                for _g in start_g..=end_g {
                    rgb_to_yuv[c as usize] = ((y << 16) + (u << 8) + v) as u32;
                    y += 1;
                    c += 0x010101;
                }
            }
        }

        rgb_to_yuv
    }

    fn pixel_difference(&self, x: usize, y: usize) -> usize {
        let yuv1 = self.rgb_to_yuv[x & 0xffffff] as usize;
        let yuv2 = self.rgb_to_yuv[y & 0xffffff] as usize;
        let yuv1y = yuv1 & Y_MASK;
        let yuv1u = yuv1 & U_MASK;
        let yuv1v = yuv1 & V_MASK;
        let yuv2y = yuv2 & Y_MASK;
        let yuv2u = yuv2 & U_MASK;
        let yuv2v = yuv2 & V_MASK;
        let x = x >> 24;
        let y = y >> 24;

        (x.abs_diff(y)) +
            (yuv1y.abs_diff(yuv2y) >> 16) +
            (yuv1u.abs_diff(yuv2u) >>  8) +
            (yuv1v.abs_diff(yuv2v))
    }

    fn similar_pixels(&self, a: usize, b: usize) -> bool {
        self.pixel_difference(a, b) < 155
    }

    fn alpha_blend_base(a: usize, b: usize, m: usize, s: usize) -> usize {
        (
            PART_MASK & (
                ((a) & PART_MASK) +
                    ((((b & PART_MASK).wrapping_sub(a & PART_MASK)) * (m)) >> (s))
            )
        ) | (
            (PART_MASK & (
                (((a) >> 8) & PART_MASK) +
                    ((((((b) >> 8) & PART_MASK).wrapping_sub((a >> 8) & PART_MASK)) * (m)) >> (s))
            )) << 8
        )
    }

    fn alpha_blend_32_w(a: u32, b: usize) -> usize {
        Self::alpha_blend_base(a as usize, b, 1, 3)
    }

    fn alpha_blend_64_w(a: u32, b: usize) -> usize {
        Self::alpha_blend_base(a as usize, b, 1, 2)
    }

    fn alpha_blend_128_w(a: u32, b: usize) -> usize {
        Self::alpha_blend_base(a as usize, b, 1, 1)
    }

    fn alpha_blend_192_w(a: u32, b: usize) -> usize {
        Self::alpha_blend_base(a as usize, b, 3, 2)
    }

    fn alpha_blend_224_w(a: u32, b: usize) -> usize {
        Self::alpha_blend_base(a as usize, b, 7, 3)
    }

    /*
     * Writes blended output values into a moving window within the output data arranged as follows:
     * |----|----|
     * |--0-|--1-|
     * |----|----|
     * |--2-|--3-|
     * |----|----|
    */
    fn filter_2(
        &self, output: &mut [u32], out_offset: usize,
        in_centre: usize, in_diag_main: usize, in_adj_cw: usize, in_adj_ccw: usize,
        in_diag_cw: usize, in_diag_ccw: usize, in_adj_cw_135: usize, in_adj_ccw_135: usize,
        in_far_ccw_45: usize, in_far_ccw_18: usize, in_far_cw_45: usize, in_far_cw_18: usize,
        out_index_1: usize, out_index_2: usize, out_index_3: usize
    ) {
        // *** Edge detection rule ***
        // Imagine a pixel at the centre of a 5x5 grid, and a line pointing diagonally out from the
        // centre pixel (say, towards the lower-right corner).

        // If the centre pixel happens to be equal to a non-diagonally adjacent pixel, perform no
        // interpolation.
        if in_centre == in_adj_cw || in_centre == in_adj_ccw {
            return;
        }

        // If a weighted difference perpendicular to that line is greater than the weighted
        // difference along it, then that is the dominant edge and this pixel must be interpolated,
        // else it need not be.
        // The 4x weight applies to the tested pair of pixels nearest to the imagined line.
        let e: usize =
            self.pixel_difference(in_centre, in_diag_ccw) + self.pixel_difference(in_centre, in_diag_cw) +
                self.pixel_difference(in_diag_main, in_far_cw_45) + self.pixel_difference(in_diag_main, in_far_ccw_45) +
                4 * self.pixel_difference(in_adj_cw, in_adj_ccw);
        let i: usize =
            self.pixel_difference(in_adj_cw, in_adj_cw_135) + self.pixel_difference(in_adj_cw, in_far_cw_18) +
                self.pixel_difference(in_adj_ccw, in_far_ccw_18) + self.pixel_difference(in_adj_ccw, in_adj_ccw_135) +
                4 * self.pixel_difference(in_centre, in_diag_main);
        if e > i {
            return;
        }

        // *** Interpolation rules ***

        // First determine select which of the two the non-diagonally adjacent pixels is the nearest
        // in colour, and then choose that to use for all blending later.
        let px: usize = if self.pixel_difference(in_centre, in_adj_ccw) <= self.pixel_difference(in_centre, in_adj_cw) { in_adj_ccw } else { in_adj_cw };

        // Check for case where no edge direction is stronger; just do 50% blending on one pixel in
        // that case
        if e == i {
            output[out_offset + out_index_3] = Self::alpha_blend_128_w(output[out_offset + out_index_3], px) as u32;
            return;
        }

        // Check various other scenarios. Where applicable, blend the area near the diagonal corner
        // in a shape that highlights the direction of the local colour gradient
        let side_parallel_gradient = !self.similar_pixels(in_adj_ccw, in_adj_ccw_135) &&
            !self.similar_pixels(in_adj_cw, in_adj_cw_135);
        let close_side_parallel_gradient = self.similar_pixels(in_centre, in_diag_main) && (
            !self.similar_pixels(in_adj_ccw, in_far_ccw_18) && !self.similar_pixels(in_adj_cw, in_far_cw_18));
        let no_perpendicular_gradient = self.similar_pixels(in_centre, in_diag_cw) ||
            self.similar_pixels(in_centre, in_diag_ccw);
        if side_parallel_gradient || close_side_parallel_gradient || no_perpendicular_gradient {
            let ke: usize = self.pixel_difference(in_adj_ccw, in_diag_cw);
            let ki: usize = self.pixel_difference(in_adj_cw, in_diag_ccw);
            let left: bool = (ke << 1) <= ki && in_centre != in_diag_cw && in_adj_cw_135 != in_diag_cw;
            let up: bool = ke >= (ki << 1) && in_centre != in_diag_ccw && in_adj_ccw_135 != in_diag_ccw;
            if left && up {
                output[out_offset + out_index_3] = Self::alpha_blend_224_w(output[out_offset + out_index_3], px) as u32;
                output[out_offset + out_index_2] = Self::alpha_blend_64_w(output[out_offset + out_index_2], px) as u32;
                output[out_offset + out_index_1] = output[out_offset + out_index_2];
            } else if left {
                output[out_offset + out_index_3] = Self::alpha_blend_192_w(output[out_offset + out_index_3], px) as u32;
                output[out_offset + out_index_2] = Self::alpha_blend_64_w(output[out_offset + out_index_2], px) as u32;
            } else if up {
                output[out_offset + out_index_3] = Self::alpha_blend_192_w(output[out_offset + out_index_3], px) as u32;
                output[out_offset + out_index_1] = Self::alpha_blend_64_w(output[out_offset + out_index_1], px) as u32;
            } else { /* diagonal */
                output[out_offset + out_index_3] = Self::alpha_blend_128_w(output[out_offset + out_index_3], px) as u32;
            }
        } else {
            output[out_offset + out_index_3] = Self::alpha_blend_128_w(output[out_offset + out_index_3], px) as u32;
        }
    }

    /*
     * Writes blended output values into a moving window within the output data arranged as follows:
     * |----|----|----|
     * |--0-|--1-|--2-|
     * |----|----|----|
     * |--3-|--4-|--5-|
     * |----|----|----|
     * |--6-|--7-|--8-|
     * |----|----|----|
    */
    fn filter_3(
        &self, output: &mut [u32], out_offset: usize,
        in_centre: usize, in_diag_main: usize, in_adj_cw: usize, in_adj_ccw: usize,
        in_diag_cw: usize, in_diag_ccw: usize, in_adj_cw_135: usize, in_adj_ccw_135: usize,
        in_far_ccw_45: usize, in_far_ccw_18: usize, in_far_cw_45: usize, in_far_cw_18: usize,
        out_index_2: usize, out_index_5: usize, out_index_6: usize, out_index_7: usize, out_index_8: usize
    ) {
        // *** Edge detection rule ***
        // Imagine a pixel at the centre of a 5x5 grid, and a line pointing diagonally out from the
        // centre pixel (say, towards the lower-right corner).

        // If the centre pixel happens to be equal to a non-diagonally adjacent pixel, perform no
        // interpolation.
        if in_centre == in_adj_cw || in_centre == in_adj_ccw {
            return;
        }

        // If a weighted difference perpendicular to that line is greater than the weighted
        // difference along it, then that is the dominant edge and this pixel must be interpolated,
        // else it need not be.
        // The 4x weight applies to the tested pair of pixels nearest to the imagined line.
        let e: usize =
            self.pixel_difference(in_centre, in_diag_ccw) + self.pixel_difference(in_centre, in_diag_cw) +
                self.pixel_difference(in_diag_main, in_far_cw_45) + self.pixel_difference(in_diag_main, in_far_ccw_45) +
                4 * self.pixel_difference(in_adj_cw, in_adj_ccw);
        let i: usize =
            self.pixel_difference(in_adj_cw, in_adj_cw_135) + self.pixel_difference(in_adj_cw, in_far_cw_18) +
                self.pixel_difference(in_adj_ccw, in_far_ccw_18) + self.pixel_difference(in_adj_ccw, in_adj_ccw_135) +
                4 * self.pixel_difference(in_centre, in_diag_main);
        if e > i {
            return;
        }

        // *** Interpolation rules ***

        // First determine select which of the two the non-diagonally adjacent pixels is the nearest
        // in colour, and then choose that to use for all blending later.
        let px: usize = if self.pixel_difference(in_centre, in_adj_ccw) <= self.pixel_difference(in_centre, in_adj_cw) { in_adj_ccw } else { in_adj_cw };

        // Check for case where no edge direction is stronger; just do 50% blending on one pixel in
        // that case
        if e == i {
            output[out_offset + out_index_8] = Self::alpha_blend_128_w(output[out_offset + out_index_8], px) as u32;
            return;
        }

        // Check various other scenarios. Where applicable, blend the area near the diagonal corner
        // in a shape that highlights the direction of the local colour gradient
        let side_parallel_gradient = !self.similar_pixels(in_adj_ccw, in_adj_ccw_135) && !self.similar_pixels(in_adj_ccw, in_diag_ccw) || !self.similar_pixels(in_adj_cw, in_adj_cw_135) && !self.similar_pixels(in_adj_cw, in_diag_cw);
        let close_side_parallel_gradient = self.similar_pixels(in_centre, in_diag_main) && (!self.similar_pixels(in_adj_ccw, in_far_ccw_45) && !self.similar_pixels(in_adj_ccw, in_far_ccw_18) || !self.similar_pixels(in_adj_cw, in_far_cw_45) && !self.similar_pixels(in_adj_cw, in_far_cw_18));
        let no_perpendicular_gradient = self.similar_pixels(in_centre, in_diag_cw) || self.similar_pixels(in_centre, in_diag_ccw);
        if side_parallel_gradient || close_side_parallel_gradient || no_perpendicular_gradient {
            let ke: usize = self.pixel_difference(in_adj_ccw, in_diag_cw);
            let ki: usize = self.pixel_difference(in_adj_cw, in_diag_ccw);
            let left: bool = (ke << 1) <= ki && in_centre != in_diag_cw && in_adj_cw_135 != in_diag_cw;
            let up: bool = ke >= (ki << 1) && in_centre != in_diag_ccw && in_adj_ccw_135 != in_diag_ccw;
            if left && up {
                output[out_offset + out_index_7] = Self::alpha_blend_192_w(output[out_offset + out_index_7], px) as u32;
                output[out_offset + out_index_6] = Self::alpha_blend_64_w(output[out_offset + out_index_6], px) as u32;
                output[out_offset + out_index_5] = output[out_offset + out_index_7];
                output[out_offset + out_index_2] = output[out_offset + out_index_6];
                output[out_offset + out_index_8] = px as u32;
            } else if left {
                output[out_offset + out_index_7] = Self::alpha_blend_192_w(output[out_offset + out_index_7], px) as u32;
                output[out_offset + out_index_5] = Self::alpha_blend_64_w(output[out_offset + out_index_5], px) as u32;
                output[out_offset + out_index_6] = Self::alpha_blend_64_w(output[out_offset + out_index_6], px) as u32;
                output[out_offset + out_index_8] = px as u32;
            } else if up {
                output[out_offset + out_index_5] = Self::alpha_blend_192_w(output[out_offset + out_index_5], px) as u32;
                output[out_offset + out_index_7] = Self::alpha_blend_64_w(output[out_offset + out_index_7], px) as u32;
                output[out_offset + out_index_2] = Self::alpha_blend_64_w(output[out_offset + out_index_2], px) as u32;
                output[out_offset + out_index_8] = px as u32;
            } else { /* diagonal */
                output[out_offset + out_index_8] = Self::alpha_blend_224_w(output[out_offset + out_index_8], px) as u32;
                output[out_offset + out_index_5] = Self::alpha_blend_32_w(output[out_offset + out_index_5], px) as u32;
                output[out_offset + out_index_7] = Self::alpha_blend_32_w(output[out_offset + out_index_7], px) as u32;
            }
        } else {
            output[out_offset + out_index_8] = Self::alpha_blend_128_w(output[out_offset + out_index_8], px) as u32;
        }
    }

    /*
     * Writes blended output values into a moving window within the output data arranged as follows:
     * |----|----|----|----|
     * |--0-|--1-|--2-|--3-|
     * |----|----|----|----|
     * |--4-|--5-|--6-|--7-|
     * |----|----|----|----|
     * |--8-|--9-|-10-|-11-|
     * |----|----|----|----|
     * |-12-|-13-|-14-|-15-|
     * |----|----|----|----|
    */
    fn filter_4(
        &self, output: &mut [u32], out_offset: usize,
        in_centre: usize, in_diag_main: usize, in_adj_cw: usize, in_adj_ccw: usize,
        in_diag_cw: usize, in_diag_ccw: usize, in_adj_cw_135: usize, in_adj_ccw_135: usize,
        in_far_ccw_45: usize, in_far_ccw_18: usize, in_far_cw_45: usize, in_far_cw_18: usize,
        out_index_15: usize, out_index_14: usize, out_index_11: usize,
        out_index_3: usize, out_index_7: usize, out_index_10: usize, out_index_13: usize, out_index_12: usize
    ) {
        // *** Edge detection rule ***
        // Imagine a pixel at the centre of a 5x5 grid, and a line pointing diagonally out from the
        // centre pixel (say, towards the lower-right corner).

        // If the centre pixel happens to be equal to a non-diagonally adjacent pixel, perform no
        // interpolation.
        if in_centre == in_adj_cw || in_centre == in_adj_ccw {
            return;
        }

        // If a weighted difference perpendicular to that line is greater than the weighted
        // difference along it, then that is the dominant edge and this pixel must be interpolated,
        // else it need not be.
        // The 4x weight applies to the tested pair of pixels nearest to the imagined line.
        let e: usize =
            self.pixel_difference(in_centre, in_diag_ccw) + self.pixel_difference(in_centre, in_diag_cw) +
                self.pixel_difference(in_diag_main, in_far_cw_45) + self.pixel_difference(in_diag_main, in_far_ccw_45) +
                4 * self.pixel_difference(in_adj_cw, in_adj_ccw);
        let i: usize =
            self.pixel_difference(in_adj_cw, in_adj_cw_135) + self.pixel_difference(in_adj_cw, in_far_cw_18) +
                self.pixel_difference(in_adj_ccw, in_far_ccw_18) + self.pixel_difference(in_adj_ccw, in_adj_ccw_135) +
                4 * self.pixel_difference(in_centre, in_diag_main);
        if e > i {
            return;
        }

        // *** Interpolation rules ***

        // First determine select which of the two the non-diagonally adjacent pixels is the nearest
        // in colour, and then choose that to use for all blending later.
        let px: usize = if self.pixel_difference(in_centre, in_adj_ccw) <= self.pixel_difference(in_centre, in_adj_cw) { in_adj_ccw } else { in_adj_cw };

        // Check for case where no edge direction is stronger; just do 50% blending on one pixel in
        // that case
        if e == i {
            output[out_offset + out_index_15] = Self::alpha_blend_128_w(output[out_offset + out_index_15], px) as u32;
            return;
        }

        // Check various other scenarios. Where applicable, blend the area near the diagonal corner
        // in a shape that highlights the direction of the local colour gradient
        let side_parallel_gradient = !self.similar_pixels(in_adj_ccw, in_adj_ccw_135) &&
            !self.similar_pixels(in_adj_cw, in_adj_cw_135);
        let close_side_parallel_gradient = self.similar_pixels(in_centre, in_diag_main) && (
            !self.similar_pixels(in_adj_ccw, in_far_ccw_18) && !self.similar_pixels(in_adj_cw, in_far_cw_18));
        let no_perpendicular_gradient = self.similar_pixels(in_centre, in_diag_cw) ||
            self.similar_pixels(in_centre, in_diag_ccw);
        if side_parallel_gradient || close_side_parallel_gradient || no_perpendicular_gradient {
            let ke: usize = self.pixel_difference(in_adj_ccw, in_diag_cw);
            let ki: usize = self.pixel_difference(in_adj_cw, in_diag_ccw);
            let left: bool = (ke << 1) <= ki && in_centre != in_diag_cw && in_adj_cw_135 != in_diag_cw;
            let up: bool = ke >= (ki << 1) && in_centre != in_diag_ccw && in_adj_ccw_135 != in_diag_ccw;
            if left & & up {
                output[out_offset + out_index_13] = Self::alpha_blend_192_w(output[out_offset + out_index_13], px) as u32;
                output[out_offset + out_index_12] = Self::alpha_blend_64_w(output[out_offset + out_index_12], px) as u32;
                output[out_offset + out_index_11] = px as u32;
                output[out_offset + out_index_14] = px as u32;
                output[out_offset + out_index_15] = px as u32;
                output[out_offset + out_index_10] = output[out_offset + out_index_12];
                output[out_offset + out_index_3] = output[out_offset + out_index_12];
                output[out_offset + out_index_7] = output[out_offset + out_index_13];
            } else if left {
                output[out_offset + out_index_11] = Self::alpha_blend_192_w(output[out_offset + out_index_11], px) as u32;
                output[out_offset + out_index_13] = Self::alpha_blend_192_w(output[out_offset + out_index_13], px) as u32;
                output[out_offset + out_index_10] = Self::alpha_blend_64_w(output[out_offset + out_index_10], px) as u32;
                output[out_offset + out_index_12] = Self::alpha_blend_64_w(output[out_offset + out_index_12], px) as u32;
                output[out_offset + out_index_14] = px as u32;
                output[out_offset + out_index_15] = px as u32;
            } else if up {
                output[out_offset + out_index_14] = Self::alpha_blend_192_w(output[out_offset + out_index_14], px) as u32;
                output[out_offset + out_index_7] = Self::alpha_blend_192_w(output[out_offset + out_index_7], px) as u32;
                output[out_offset + out_index_10] = Self::alpha_blend_64_w(output[out_offset + out_index_10], px) as u32;
                output[out_offset + out_index_3] = Self::alpha_blend_64_w(output[out_offset + out_index_3], px) as u32;
                output[out_offset + out_index_11] = px as u32;
                output[out_offset + out_index_15] = px as u32;
            } else { /* diagonal */
                output[out_offset + out_index_11] = Self::alpha_blend_128_w(output[out_offset + out_index_11], px) as u32;
                output[out_offset + out_index_14] = Self::alpha_blend_128_w(output[out_offset + out_index_14], px) as u32;
                output[out_offset + out_index_15] = px as u32;
            }
        } else {
            output[out_offset + out_index_15] = Self::alpha_blend_128_w(output[out_offset + out_index_15], px) as u32;
        }
    }

    /*
     * Works with a grid of values taken from the input data with arrangement as follows:
     * |----|----|----|----|----|
     * |----|-b0-|-c0-|-d0-|----|
     * |----|----|----|----|----|
     * |-a1-|-b1-|-c1-|-d1-|-e1-|
     * |----|----|----|----|----|
     * |-a2-|-b2-|-c2-|-d2-|-e2-|
     * |----|----|----|----|----|
     * |-a3-|-b3-|-c3-|-d3-|-e3-|
     * |----|----|----|----|----|
     * |----|-b4-|-c4-|-d4-|----|
     * |----|----|----|----|----|
    */
    fn xbr_filter(&mut self, n: usize) {

        let out_offset_1_line = self.input_width * n;
        let out_offset_2_lines = self.input_width * n * 2;
        let out_offset_3_lines = self.input_width * n * 3;

        let mut input = self.input.borrow_mut();
        let input_data = input.as_mut_slice();
        let mut output = self.output.borrow_mut();
        let output_data = output.as_mut_slice();

        for y in 0..self.input_height {

            let in_up_2_row_offset: usize = (y.max(2) - 2) * self.input_width;
            let in_up_1_row_offset: usize = (y.max(1) - 1) * self.input_width;
            let in_cent_row_offset: usize = y * self.input_width;
            let in_dn_1_row_offset: usize = (y.min(self.input_height - 2) + 1) * self.input_width;
            let in_dn_2_row_offset: usize = (y.min(self.input_height - 3) + 2) * self.input_width;

            for x in 0..self.input_width {

                let mut out_offset = (y * self.input_width + x) * n;

                let in_lf_2_offset: usize = x.max(2) - 2;
                let in_lf_1_offset: usize = x.max(1) - 1;
                let in_rt_1_offset: usize = x.min(self.input_width - 2) + 1;
                let in_rt_2_offset: usize = x.min(self.input_width - 3) + 2;

                let in_a1 = input_data[in_up_1_row_offset + in_lf_2_offset] as usize;
                let in_a2 = input_data[in_cent_row_offset + in_lf_2_offset] as usize;
                let in_a3 = input_data[in_dn_1_row_offset + in_lf_2_offset] as usize;

                let in_b0 = input_data[in_up_2_row_offset + in_lf_1_offset] as usize;
                let in_b1 = input_data[in_up_1_row_offset + in_lf_1_offset] as usize;
                let in_b2 = input_data[in_cent_row_offset + in_lf_1_offset] as usize;
                let in_b3 = input_data[in_dn_1_row_offset + in_lf_1_offset] as usize;
                let in_b4 = input_data[in_dn_2_row_offset + in_lf_1_offset] as usize;

                let in_c0 = input_data[in_up_2_row_offset] as usize;
                let in_c1 = input_data[in_up_1_row_offset] as usize;
                let in_c2 = input_data[in_cent_row_offset] as usize;
                let in_c3 = input_data[in_dn_1_row_offset] as usize;
                let in_c4 = input_data[in_dn_2_row_offset] as usize;

                let in_d0 = input_data[in_up_2_row_offset + in_rt_1_offset] as usize;
                let in_d1 = input_data[in_up_1_row_offset + in_rt_1_offset] as usize;
                let in_d2 = input_data[in_cent_row_offset + in_rt_1_offset] as usize;
                let in_d3 = input_data[in_dn_1_row_offset + in_rt_1_offset] as usize;
                let in_d4 = input_data[in_dn_2_row_offset + in_rt_1_offset] as usize;

                let in_e1 = input_data[in_up_1_row_offset + in_rt_2_offset] as usize;
                let in_e2 = input_data[in_cent_row_offset + in_rt_2_offset] as usize;
                let in_e3 = input_data[in_dn_1_row_offset + in_rt_2_offset] as usize;

                match n {
                    2 => {
                        output_data[out_offset + 0] = in_c2 as u32; // 0
                        output_data[out_offset + 1] = in_c2 as u32; // 1
                        output_data[out_offset + out_offset_1_line] = in_c2 as u32; // 2
                        output_data[out_offset + out_offset_1_line + 1] = in_c2 as u32; // 3

                        let nl_1 = out_offset_1_line + 1;
                        self.filter_2(output_data, out_offset, in_c2, in_d3, in_c3, in_d2, in_b3, in_d1, in_b2, in_c1, in_e2, in_e3, in_c4, in_d4, CONST_ONE, out_offset_1_line, nl_1);
                        self.filter_2(output_data, out_offset, in_c2, in_d1, in_d2, in_c1, in_d3, in_b1, in_c3, in_b2, in_c0, in_d0, in_e2, in_e1, CONST_ZERO, nl_1, CONST_ONE);
                        self.filter_2(output_data, out_offset, in_c2, in_b1, in_c1, in_b2, in_d1, in_b3, in_d2, in_c3, in_a2, in_a1, in_c0, in_b0, out_offset_1_line, CONST_ONE, CONST_ZERO);
                        self.filter_2(output_data, out_offset, in_c2, in_b3, in_b2, in_c3, in_b1, in_d3, in_c1, in_d2, in_c4, in_b4, in_a2, in_a3, nl_1, CONST_ZERO, out_offset_1_line);
                    },
                    3 => {
                        output_data[out_offset + 0] = in_c2 as u32; // 0
                        output_data[out_offset + 1] = in_c2 as u32; // 1
                        output_data[out_offset + 2] = in_c2 as u32; // 2
                        output_data[out_offset + out_offset_1_line] = in_c2 as u32; // 3
                        output_data[out_offset + out_offset_1_line + 1] = in_c2 as u32; // 4
                        output_data[out_offset + out_offset_1_line + 2] = in_c2 as u32; // 5
                        output_data[out_offset + out_offset_2_lines] = in_c2 as u32; // 6
                        output_data[out_offset + out_offset_2_lines + 1] = in_c2 as u32; // 7
                        output_data[out_offset + out_offset_2_lines + 2] = in_c2 as u32; // 8

                        let nl_1 = out_offset_1_line + 1;
                        let nl_2 = out_offset_1_line + 2;
                        let nl1_1 = out_offset_2_lines + 1;
                        let nl1_2 = out_offset_2_lines + 2;
                        self.filter_3(output_data, out_offset, in_c2, in_d3, in_c3, in_d2, in_b3, in_d1, in_b2, in_c1, in_e2, in_e3, in_c4, in_d4, CONST_TWO, nl_2, out_offset_2_lines, nl1_1, nl1_2);
                        self.filter_3(output_data, out_offset, in_c2, in_d1, in_d2, in_c1, in_d3, in_b1, in_c3, in_b2, in_c0, in_d0, in_e2, in_e1, CONST_ZERO, CONST_ONE, nl1_2, nl_2, CONST_TWO);
                        self.filter_3(output_data, out_offset, in_c2, in_b1, in_c1, in_b2, in_d1, in_b3, in_d2, in_c3, in_a2, in_a1, in_c0, in_b0, out_offset_2_lines, out_offset_1_line, CONST_TWO, CONST_ONE, CONST_ZERO);
                        self.filter_3(output_data, out_offset, in_c2, in_b3, in_b2, in_c3, in_b1, in_d3, in_c1, in_d2, in_c4, in_b4, in_a2, in_a3, nl1_2, nl1_1, CONST_ZERO, out_offset_1_line, out_offset_2_lines);
                    },
                    4 => {
                        output_data[out_offset + 0] = in_c2 as u32; // 0
                        output_data[out_offset + 1] = in_c2 as u32; // 1
                        output_data[out_offset + 2] = in_c2 as u32; // 2
                        output_data[out_offset + 3] = in_c2 as u32; // 3
                        output_data[out_offset + out_offset_1_line] = in_c2 as u32; // 4
                        output_data[out_offset + out_offset_1_line + 1] = in_c2 as u32; // 5
                        output_data[out_offset + out_offset_1_line + 2] = in_c2 as u32; // 6
                        output_data[out_offset + out_offset_1_line + 3] = in_c2 as u32; // 7
                        output_data[out_offset + out_offset_2_lines] = in_c2 as u32; // 8
                        output_data[out_offset + out_offset_2_lines + 1] = in_c2 as u32; // 9
                        output_data[out_offset + out_offset_2_lines + 2] = in_c2 as u32; // 10
                        output_data[out_offset + out_offset_2_lines + 3] = in_c2 as u32; // 11
                        output_data[out_offset + out_offset_3_lines] = in_c2 as u32; // 12
                        output_data[out_offset + out_offset_3_lines + 1] = in_c2 as u32; // 13
                        output_data[out_offset + out_offset_3_lines + 2] = in_c2 as u32; // 14
                        output_data[out_offset + out_offset_3_lines + 3] = in_c2 as u32; // 15

                        let nl_1 = out_offset_1_line + 1;
                        let nl_2 = out_offset_1_line + 2;
                        let nl_3 = out_offset_1_line + 3;
                        let nl1_1 = out_offset_2_lines + 1;
                        let nl1_2 = out_offset_2_lines + 2;
                        let nl1_3 = out_offset_2_lines + 3;
                        let nl2_1 = out_offset_3_lines + 1;
                        let nl2_2 = out_offset_3_lines + 2;
                        let nl2_3 = out_offset_3_lines + 3;
                        self.filter_4(output_data, out_offset, in_c2, in_d3, in_c3, in_d2, in_b3, in_d1, in_b2, in_c1, in_e2, in_e3, in_c4, in_d4, nl2_3, nl2_2, nl1_3, CONST_THREE, nl_3, nl1_2, nl2_1, out_offset_3_lines);
                        self.filter_4(output_data, out_offset, in_c2, in_d1, in_d2, in_c1, in_d3, in_b1, in_c3, in_b2, in_c0, in_d0, in_e2, in_e1, CONST_THREE, nl_3, CONST_TWO, CONST_ZERO, CONST_ONE, nl_2, nl1_3, nl2_3);
                        self.filter_4(output_data, out_offset, in_c2, in_b1, in_c1, in_b2, in_d1, in_b3, in_d2, in_c3, in_a2, in_a1, in_c0, in_b0, CONST_ZERO, CONST_ONE, out_offset_1_line, out_offset_3_lines, out_offset_2_lines, nl_1, CONST_TWO, CONST_THREE);
                        self.filter_4(output_data, out_offset, in_c2, in_b3, in_b2, in_c3, in_b1, in_d3, in_c1, in_d2, in_c4, in_b4, in_a2, in_a3, out_offset_3_lines, out_offset_2_lines, nl2_1, nl2_3, nl2_2, nl1_1, out_offset_1_line, CONST_ZERO);
                    },
                    _ => {}
                }

                out_offset += n;
            }
        }
    }

    pub fn xbr_filter_xbr2x(&mut self) {
        self.xbr_filter(2);
    }

    pub fn xbr_filter_xbr3x(&mut self) {
        self.xbr_filter(3);
    }

    pub fn xbr_filter_xbr4x(&mut self) {
        self.xbr_filter(4);
    }
}
