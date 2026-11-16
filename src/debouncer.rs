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

    pub fn for_each_falling_edge<F>(&self, f: F)
    where
        F: FnMut(u8),
    {
        self.for_each_bit(self.prev_stable_state & !self.stable_state, f);
    }

    pub fn for_each_rising_edge<F>(&self, f: F)
    where
        F: FnMut(u8),
    {
        self.for_each_bit(!self.prev_stable_state & self.stable_state, f);
    }

    fn for_each_bit<F>(&self, mut mask: u16, mut f: F)
    where
        F: FnMut(u8),
    {
        while mask != 0 {
            let i = mask.trailing_zeros() as u8;
            f(i);
            mask &= mask - 1;
        }
    }
}
