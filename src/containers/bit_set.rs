//! Bit set provides fast operations on large arrays of bits.

use crate::base::core::b3_assert;

#[derive(Clone, Debug, Default)]
pub struct BitSet {
    bits: Vec<u64>,
    block_capacity: u32,
    block_count: u32,
}

impl BitSet {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_bit_capacity(bit_capacity: u32) -> Self {
        let block_capacity = bit_capacity.div_ceil(64);
        Self {
            bits: vec![0; block_capacity as usize],
            block_capacity,
            block_count: 0,
        }
    }

    pub fn destroy(&mut self) {
        self.bits.clear();
        self.block_capacity = 0;
        self.block_count = 0;
    }

    pub fn set_bit_count_and_clear(&mut self, bit_count: u32) {
        let block_count = bit_count.div_ceil(64);
        if self.block_capacity < block_count {
            let new_bit_capacity = bit_count + (bit_count >> 1);
            *self = Self::with_bit_capacity(new_bit_capacity);
        }

        self.block_count = block_count;
        self.bits[..block_count as usize].fill(0);
    }

    pub fn grow(&mut self, block_count: u32) {
        b3_assert(block_count > self.block_count);
        if block_count > self.block_capacity {
            let old_capacity = self.block_capacity;
            self.block_capacity = block_count + block_count / 2;
            self.bits.resize(self.block_capacity as usize, 0);
            let _ = old_capacity;
        }
        self.block_count = block_count;
    }

    pub fn in_place_union(&mut self, set_b: &BitSet) {
        b3_assert(self.block_count == set_b.block_count);
        for i in 0..self.block_count as usize {
            self.bits[i] |= set_b.bits[i];
        }
    }

    #[inline(always)]
    pub fn set_bit(&mut self, bit_index: u32) {
        let block_index = (bit_index / 64) as usize;
        b3_assert((block_index as u32) < self.block_count);
        self.bits[block_index] |= 1u64 << (bit_index % 64);
    }

    #[inline(always)]
    pub fn set_bit_grow(&mut self, bit_index: u32) {
        let block_index = bit_index / 64;
        if block_index >= self.block_count {
            self.grow(block_index + 1);
        }
        self.bits[block_index as usize] |= 1u64 << (bit_index % 64);
    }

    #[inline(always)]
    pub fn clear_bit(&mut self, bit_index: u32) {
        let block_index = bit_index / 64;
        if block_index >= self.block_count {
            return;
        }
        self.bits[block_index as usize] &= !(1u64 << (bit_index % 64));
    }

    /// Visit set bits in ascending order without scanning the whole bit count.
    #[inline(always)]
    pub fn for_each_set_bit<F: FnMut(i32)>(&self, mut body: F) {
        for block in 0..self.block_count as usize {
            let mut word = self.bits[block];
            while word != 0 {
                let lane = word.trailing_zeros();
                body((block << 6) as i32 + lane as i32);
                word &= word - 1;
            }
        }
    }

    #[inline(always)]
    pub fn get_bit(&self, bit_index: u32) -> bool {
        let block_index = bit_index / 64;
        if block_index >= self.block_count {
            return false;
        }
        (self.bits[block_index as usize] & (1u64 << (bit_index % 64))) != 0
    }

    pub fn count_set_bits(&self) -> i32 {
        let mut pop_count = 0;
        for i in 0..self.block_count as usize {
            pop_count += self.bits[i].count_ones() as i32;
        }
        pop_count
    }

    pub fn byte_count(&self) -> i32 {
        self.block_capacity as i32 * std::mem::size_of::<u64>() as i32
    }

    pub fn block_count(&self) -> u32 {
        self.block_count
    }
}
