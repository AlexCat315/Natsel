//! A growable array of plain values with stable element addresses.

#[derive(Clone, Debug)]
pub struct StableStorage<T: Copy + Default> {
    base: Vec<T>,
}

impl<T: Copy + Default> Default for StableStorage<T> {
    fn default() -> Self {
        Self::new(16)
    }
}

impl<T: Copy + Default> StableStorage<T> {
    pub fn new(capacity: usize) -> Self {
        Self {
            base: Vec::with_capacity(capacity.max(1)),
        }
    }

    pub fn append(&mut self, value: T) {
        self.base.push(value);
    }

    pub fn count(&self) -> usize {
        self.base.len()
    }

    pub fn get(&self, index: usize) -> T {
        crate::base::core::b3_assert(index < self.base.len());
        self.base[index]
    }

    pub fn set(&mut self, index: usize, value: T) {
        crate::base::core::b3_assert(index < self.base.len());
        self.base[index] = value;
    }

    /// The elements as a slice, for snapshots.
    pub fn elements(&self) -> &[T] {
        &self.base
    }

    /// Replace all elements. Only on one thread with no readers.
    pub fn replace(&mut self, elements: &[T]) {
        self.base.clear();
        self.base.extend_from_slice(elements);
    }

    pub fn as_ptr(&self) -> *const T {
        self.base.as_ptr()
    }

    pub fn as_mut_ptr(&mut self) -> *mut T {
        self.base.as_mut_ptr()
    }

    pub fn as_slice(&self) -> &[T] {
        &self.base
    }

    pub fn as_mut_slice(&mut self) -> &mut [T] {
        &mut self.base
    }
}
