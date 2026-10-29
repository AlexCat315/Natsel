//! A growable array with explicit storage. Elements live in contiguous storage owned by the
//! array. Pointers returned by [`DynamicArray::pointer`] stay valid until the array grows.

use crate::base::constants;
use crate::base::core::b3_assert;

#[derive(Clone, Debug)]
pub struct DynamicArray<T: Copy + Default> {
    data: Vec<T>,
}

impl<T: Copy + Default> Default for DynamicArray<T> {
    fn default() -> Self {
        Self { data: Vec::new() }
    }
}

impl<T: Copy + Default> DynamicArray<T> {
    pub fn new() -> Self {
        Self::default()
    }

    /// Create with an initial capacity.
    pub fn with_capacity(capacity: i32) -> Self {
        if capacity > 0 {
            Self {
                data: vec![T::default(); capacity as usize],
            }
        } else {
            Self::default()
        }
    }

    pub fn destroy(&mut self) {
        self.data = Vec::new();
    }

    pub fn reserve(&mut self, n: i32) {
        let n = n as usize;
        if self.data.capacity() < n {
            self.data.reserve(n - self.data.len());
        }
    }

    /// Set the count, filling new elements with the default value.
    pub fn resize(&mut self, n: i32) {
        let n = n as usize;
        if n > self.data.len() {
            self.data.resize(n, T::default());
        } else {
            self.data.truncate(n);
        }
    }

    #[inline(always)]
    pub fn push(&mut self, value: T) {
        self.data.push(value);
    }

    /// Get a raw pointer to an element.
    #[inline(always)]
    pub fn pointer(&mut self, index: i32) -> *mut T {
        b3_assert(index >= 0 && index < self.count());
        unsafe { self.data.as_mut_ptr().add(index as usize) }
    }

    #[inline(always)]
    pub fn count(&self) -> i32 {
        self.data.len() as i32
    }

    pub fn as_slice(&self) -> &[T] {
        &self.data
    }

    pub fn as_mut_slice(&mut self) -> &mut [T] {
        &mut self.data
    }

    /// Create a new default element and return a pointer to it.
    #[inline(always)]
    pub fn emplace(&mut self) -> *mut T {
        self.data.push(T::default());
        let index = self.data.len() - 1;
        unsafe { self.data.as_mut_ptr().add(index) }
    }

    /// Add a default element and return its index.
    #[inline(always)]
    pub fn add_index(&mut self) -> i32 {
        self.data.push(T::default());
        (self.data.len() - 1) as i32
    }

    /// Remove the last element and return it by value.
    #[inline(always)]
    pub fn pop(&mut self) -> T {
        b3_assert(!self.data.is_empty());
        self.data.pop().unwrap()
    }

    /// Append a contiguous run of values.
    pub fn append(&mut self, src: &[T]) {
        self.data.extend_from_slice(src);
    }

    /// Zero the entire allocated buffer (capacity, not just count).
    pub fn mem_zero(&mut self) {
        for value in &mut self.data {
            *value = T::default();
        }
    }

    /// Remove an element by swapping with the last element. If the index is the last element
    /// it returns `NULL_INDEX`, otherwise it returns the index of the last element (which is now
    /// out of bounds).
    #[inline(always)]
    pub fn remove_swap(&mut self, index: i32) -> i32 {
        b3_assert_msg(index >= 0 && index < self.count(), "Array index out of bounds");
        let last = self.data.len() - 1;
        self.data.swap(index as usize, last);
        self.data.pop();
        if index as usize != last {
            last as i32
        } else {
            constants::NULL_INDEX
        }
    }

    #[inline(always)]
    pub fn clear(&mut self) {
        self.data.clear();
    }

    pub fn byte_count(&self) -> i32 {
        (self.data.capacity() * std::mem::size_of::<T>()) as i32
    }

    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    #[inline(always)]
    pub fn get(&self, index: i32) -> T {
        self.data[index as usize]
    }

    #[inline(always)]
    pub fn set(&mut self, index: i32, value: T) {
        self.data[index as usize] = value;
    }

    /// Get a copy of an element.
    #[inline(always)]
    pub fn get_index(&self, index: usize) -> T {
        self.data[index]
    }

    pub fn iter(&self) -> std::slice::Iter<'_, T> {
        self.data.iter()
    }
}

impl<T: Copy + Default> std::ops::Index<usize> for DynamicArray<T> {
    type Output = T;
    #[inline(always)]
    fn index(&self, index: usize) -> &T {
        &self.data[index]
    }
}

impl<T: Copy + Default> std::ops::IndexMut<usize> for DynamicArray<T> {
    #[inline(always)]
    fn index_mut(&mut self, index: usize) -> &mut T {
        &mut self.data[index]
    }
}
