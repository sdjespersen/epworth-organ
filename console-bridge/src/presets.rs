pub struct PresetStore {
    presets: [[u16; 4]; 8],
}

impl PresetStore {
    pub fn new() -> Self {
        Self {
            presets: [[0; 4]; 8],
        }
    }

    pub fn save(&mut self, key: u8, value: &[u16; 4]) {
        for i in 0..4 {
            self.presets[key as usize][i] = value[i];
        }
    }

    pub fn load(&self, key: u8) -> &[u16; 4] {
        &self.presets[key as usize]
    }
}
