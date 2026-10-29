//! Deterministic interning of shared child geometry and materials.

use std::collections::HashMap;

use crate::collision::geometry_storage::GeometryStorage;

/// Insertion order is explicit; hash-table iteration never determines serialized order.
#[derive(Clone, Debug, Default)]
pub struct CompoundInternTable<T> {
    indices: HashMap<Vec<u8>, usize>,
    pub values: Vec<T>,
}

impl<T> CompoundInternTable<T> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn insert(&mut self, value: T, bytes: Vec<u8>) -> usize {
        if let Some(index) = self.indices.get(&bytes) {
            return *index;
        }
        let index = self.values.len();
        self.indices.insert(bytes, index);
        self.values.push(value);
        index
    }
}

/// Copy the bytes of a geometry allocation.
pub fn geometry_bytes(storage: &GeometryStorage) -> Vec<u8> {
    let byte_count = storage.byte_count();
    unsafe { std::slice::from_raw_parts(storage.base() as *const u8, byte_count).to_vec() }
}
