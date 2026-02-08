struct BitsWhere {
    n: u16,
}

impl Iterator for BitsWhere {
    type Item = u8;

    fn next(&mut self) -> Option<Self::Item> {
        if self.n == 0 {
            return None;
        }

        // Find the index of the lowest set bit (0-15)
        let index = self.n.trailing_zeros();

        // Clear the lowest set bit (Kernighan's bit counting trick)
        self.n &= self.n - 1;

        Some(index as u8)
    }
}

pub struct Debouncer<const DEPTH: usize> {
    history: [u16; DEPTH],
    index: usize,
    stable_state: u16,
    prev_stable_state: u16,
}

impl<const DEPTH: usize> Debouncer<DEPTH> {
    pub fn new(initial: u16) -> Self {
        Self {
            history: [initial; DEPTH],
            index: 0,
            stable_state: initial,
            prev_stable_state: initial,
        }
    }

    pub fn update(&mut self, raw_reading: u16) {
        self.history[self.index] = raw_reading;
        self.index = (self.index + 1) % DEPTH;

        let mut stable_high = !(0 as u16);
        let mut stable_low = 0 as u16;

        for val in self.history.iter() {
            stable_high &= *val;
            stable_low |= *val;
        }

        self.prev_stable_state = self.stable_state;
        self.stable_state |= stable_high;
        self.stable_state &= stable_low;
    }

    pub fn falling_edges(&self) -> impl Iterator<Item = u8> {
        BitsWhere {
            n: self.prev_stable_state & !self.stable_state,
        }
    }

    pub fn rising_edges(&self) -> impl Iterator<Item = u8> {
        BitsWhere {
            n: !self.prev_stable_state & self.stable_state,
        }
    }
}
