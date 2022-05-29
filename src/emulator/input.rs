
pub struct Input {
    pub key_dir: u8,
    pub key_but: u8
}

impl Input {

    pub fn new() -> Self {
        let mut input = Self {
            key_dir: 0x0f,
            key_but: 0x0f
        };

        // Return object ready to run
        input.reset();
        input
    }

    pub fn reset(&mut self) {
        self.key_dir = 0x0f;
        self.key_but = 0x0f;
    }

    pub fn press_right(&mut self) {
        self.key_dir &= 0x0e;
    }

    pub fn press_left(&mut self) {
        self.key_dir &= 0x0d;
    }

    pub fn press_up(&mut self) {
        self.key_dir &= 0x0b;
    }

    pub fn press_down(&mut self) {
        self.key_dir &= 0x07;
    }

    pub fn press_select(&mut self) {
        self.key_but &= 0x0b;
    }

    pub fn press_start(&mut self) {
        self.key_but &= 0x07;
    }

    pub fn press_a(&mut self) {
        self.key_but &= 0x0e;
    }

    pub fn press_b(&mut self) {
        self.key_but &= 0x0d;
    }

    pub fn release_right(&mut self) {
        self.key_dir |= 0x01;
    }

    pub fn release_left(&mut self) {
        self.key_dir |= 0x02;
    }

    pub fn release_up(&mut self) {
        self.key_dir |= 0x04;
    }

    pub fn release_down(&mut self) {
        self.key_dir |= 0x08;
    }

    pub fn release_select(&mut self) {
        self.key_but |= 0x04;
    }

    pub fn release_start(&mut self) {
        self.key_but |= 0x08;
    }

    pub fn release_a(&mut self) {
        self.key_but |= 0x01;
    }

    pub fn release_b(&mut self) {
        self.key_but |= 0x02;
    }
}
