//! Open addressing hash set of non-zero 64-bit keys.

use crate::base::constants;
use crate::base::core::b3_assert;
use crate::containers::bits::round_up_power_of_2;

#[inline(always)]
pub fn key_hash(key: u64) -> u32 {
    let mut h = key;
    h ^= h >> 33;
    h = h.wrapping_mul(0xff51_afd7_ed55_8ccd);
    h ^= h >> 33;
    h = h.wrapping_mul(0xc4ce_b9fe_1a85_ec53);
    h ^= h >> 33;
    h as u32
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SetItem {
    pub key: u64,
    pub hash: u32,
}

#[inline(always)]
pub fn shape_pair_key(s1: i32, s2: i32, c: i32) -> u64 {
    let shape_mask = (constants::MAX_SHAPES - 1) as u64;
    let child_mask = (constants::MAX_CHILD_SHAPES - 1) as u64;
    let child = (c as i64 as u64) & child_mask;
    let a = shape_mask & (s1 as i64 as u64);
    let b = shape_mask & (s2 as i64 as u64);
    if s1 < s2 {
        (a << (64 - constants::SHAPE_POWER))
            | (b << (64 - 2 * constants::SHAPE_POWER))
            | child
    } else {
        (b << (64 - constants::SHAPE_POWER))
            | (a << (64 - 2 * constants::SHAPE_POWER))
            | child
    }
}

#[derive(Clone, Debug)]
pub struct HashSet {
    items: Vec<SetItem>,
    capacity: u32,
    count: u32,
}

impl Default for HashSet {
    fn default() -> Self {
        Self {
            items: Vec::new(),
            capacity: 0,
            count: 0,
        }
    }
}

impl HashSet {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_capacity(capacity: i32) -> Self {
        let capacity = if capacity > 16 {
            round_up_power_of_2(capacity) as u32
        } else {
            16
        };
        Self {
            items: vec![SetItem::default(); capacity as usize],
            capacity,
            count: 0,
        }
    }

    pub fn destroy(&mut self) {
        self.items.clear();
        self.capacity = 0;
        self.count = 0;
    }

    pub fn clear(&mut self) {
        self.count = 0;
        self.items.fill(SetItem::default());
    }

    #[inline(always)]
    fn find_slot(&self, key: u64, hash: u32) -> usize {
        let capacity = self.capacity;
        let mut index = (hash & (capacity - 1)) as usize;
        while self.items[index].hash != 0 && self.items[index].key != key {
            index = (index + 1) & (capacity - 1) as usize;
        }
        index
    }

    fn add_key_have_capacity(&mut self, key: u64, hash: u32) {
        let index = self.find_slot(key, hash);
        b3_assert(self.items[index].hash == 0);
        self.items[index].key = key;
        self.items[index].hash = hash;
        self.count += 1;
    }

    fn grow(&mut self) {
        let old_count = self.count;
        let old_capacity = self.capacity;
        let old_items = std::mem::take(&mut self.items);

        self.count = 0;
        self.capacity = 2 * old_capacity;
        self.items = vec![SetItem::default(); self.capacity as usize];

        for item in &old_items {
            if item.hash != 0 {
                self.add_key_have_capacity(item.key, item.hash);
            }
        }

        b3_assert(self.count == old_count);
    }

    pub fn contains_key(&self, key: u64) -> bool {
        b3_assert(key != 0);
        let hash = key_hash(key);
        let index = self.find_slot(key, hash);
        self.items[index].key == key
    }

    /// The caller already has the hash, which lets a batch prefetch the slots before probing.
    pub fn contains_hashed_key(&self, key: u64, hash: u32) -> bool {
        b3_assert(key != 0);
        b3_assert(hash == key_hash(key));
        let index = self.find_slot(key, hash);
        self.items[index].key == key
    }

    pub fn byte_count(&self) -> i32 {
        self.capacity as i32 * std::mem::size_of::<SetItem>() as i32
    }

    pub fn count(&self) -> u32 {
        self.count
    }

    /// Returns true if `key` was already in the set.
    pub fn add_key(&mut self, key: u64) -> bool {
        b3_assert(key != 0);
        let hash = key_hash(key);
        b3_assert(hash != 0);

        let index = self.find_slot(key, hash);
        if self.items[index].hash != 0 {
            b3_assert(self.items[index].hash == hash && self.items[index].key == key);
            return true;
        }

        if 2 * self.count >= self.capacity {
            self.grow();
        }

        self.add_key_have_capacity(key, hash);
        false
    }

    /// Returns true if the key was found.
    pub fn remove_key(&mut self, key: u64) -> bool {
        let hash = key_hash(key);
        let mut i = self.find_slot(key, hash);
        if self.items[i].hash == 0 {
            return false;
        }

        self.items[i].key = 0;
        self.items[i].hash = 0;
        b3_assert(self.count > 0);
        self.count -= 1;

        let mut j = i;
        let capacity = self.capacity;
        loop {
            j = (j + 1) & (capacity - 1) as usize;
            if self.items[j].hash == 0 {
                break;
            }

            let k = (self.items[j].hash & (capacity - 1)) as usize;

            if i <= j {
                if i < k && k <= j {
                    continue;
                }
            } else if i < k || k <= j {
                continue;
            }

            self.items[i] = self.items[j];
            self.items[j].key = 0;
            self.items[j].hash = 0;
            i = j;
        }

        true
    }
}
