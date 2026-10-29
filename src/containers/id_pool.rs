//! An index pool with free list.

use crate::base::core::{b3_assert, B3_ENABLE_VALIDATION};
use crate::containers::dynamic_array::DynamicArray;

#[derive(Clone, Debug, Default)]
pub struct IDPool {
    pub free_array: DynamicArray<i32>,
    pub next_index: i32,
}

impl IDPool {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn create() -> Self {
        let mut pool = IDPool::new();
        pool.free_array.reserve(32);
        pool
    }

    pub fn destroy(&mut self) {
        self.free_array.destroy();
        *self = IDPool::new();
    }

    pub fn alloc(&mut self) -> i32 {
        if self.free_array.count() > 0 {
            return self.free_array.pop();
        }
        let id = self.next_index;
        self.next_index += 1;
        id
    }

    pub fn free(&mut self, id: i32) {
        b3_assert(self.next_index > 0);
        b3_assert(id >= 0 && id < self.next_index);

        if id == self.next_index {
            self.next_index -= 1;
            return;
        }

        self.free_array.push(id);
    }

    pub fn validate_free(&self, id: i32) {
        if !B3_ENABLE_VALIDATION {
            return;
        }
        for i in 0..self.free_array.count() {
            if self.free_array.get(i) == id {
                return;
            }
        }
        b3_assert(false);
    }

    pub fn id_count(&self) -> i32 {
        self.next_index - self.free_array.count()
    }

    pub fn id_capacity(&self) -> i32 {
        self.next_index
    }

    pub fn byte_count(&self) -> i32 {
        self.free_array.byte_count()
    }
}
