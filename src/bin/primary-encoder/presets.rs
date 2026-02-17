use core::ops::Range;
use embassy_rp::flash::{Async, ERASE_SIZE, Flash, PAGE_SIZE};
use embassy_rp::peripherals::FLASH;
use epworth_organ::encoder::PresetStore;
use sequential_storage::cache::NoCache;
use sequential_storage::map::{MapConfig, MapStorage};

// Raspberry Pi Pico 2 flash size is 4MB
const FLASH_SIZE: usize = 4 * 1024 * 1024;
// We will use the last 9 erase blocks, except for the very last one which is reserved for the bootloader. This gives
// us 32KB of storage for presets, which will be way more than enough.
const FLASH_OFFSET: u32 = (FLASH_SIZE - (9 * ERASE_SIZE)) as u32;
const END_PRESET_FLASH: u32 = (FLASH_SIZE - ERASE_SIZE) as u32;
const PRESET_FLASH_RANGE: Range<u32> = FLASH_OFFSET..END_PRESET_FLASH;

pub struct Presets<'d> {
    storage: MapStorage<u8, Flash<'d, FLASH, Async, FLASH_SIZE>, NoCache>,
}

impl<'d> Presets<'d> {
    pub fn new(flash: Flash<'d, FLASH, Async, FLASH_SIZE>) -> Self {
        Self {
            storage: MapStorage::<u8, Flash<'d, FLASH, Async, FLASH_SIZE>, NoCache>::new(
                flash,
                const { MapConfig::new(PRESET_FLASH_RANGE) },
                NoCache::new(),
            ),
        }
    }
}

impl<'d> PresetStore for Presets<'d> {
    async fn save(&mut self, key: u8, value: u64) {
        let mut data_buffer = [0u8; PAGE_SIZE];
        self.storage
            .store_item(&mut data_buffer, &key, &value)
            .await
            .unwrap();
    }

    async fn load(&mut self, key: u8) -> u64 {
        let mut data_buffer = [0u8; PAGE_SIZE];
        self
            .storage
            .fetch_item::<u64>(&mut data_buffer, &key)
            .await
            .unwrap()
            .unwrap_or(0u64)
    }
}
