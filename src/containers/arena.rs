//! Stack-like arena allocator and bump allocator with shared overflow tracking.

use crate::base::constants;
use crate::base::core::*;
use crate::containers::dynamic_array::DynamicArray;

const MAX_STACK_ENTRIES: usize = 32;

#[derive(Clone, Copy)]
pub struct StackEntry {
    pub data: *mut u8,
    pub size: i32,
    pub used_malloc: bool,
}

impl Default for StackEntry {
    fn default() -> Self {
        Self {
            data: std::ptr::null_mut(),
            size: 0,
            used_malloc: false,
        }
    }
}

/// This is a stack-like arena allocator used for fast per step allocations.
/// You must nest allocate/free pairs. This allocator uses the heap if space is insufficient.
#[derive(Clone)]
pub struct ArenaStack {
    pub memory: *mut u8,
    pub capacity: i32,
    pub index: i32,
    pub allocation: i32,
    pub max_allocation: i32,
    pub entries: [StackEntry; MAX_STACK_ENTRIES],
    pub entry_count: i32,
}

impl Default for ArenaStack {
    fn default() -> Self {
        Self {
            memory: std::ptr::null_mut(),
            capacity: 0,
            index: 0,
            allocation: 0,
            max_allocation: 0,
            entries: [StackEntry::default(); MAX_STACK_ENTRIES],
            entry_count: 0,
        }
    }
}

impl ArenaStack {
    pub fn new(capacity: i32) -> Self {
        b3_assert(capacity >= 0);
        let mut stack = Self::default();
        stack.capacity = capacity;
        stack.memory = b3_alloc(capacity as usize);
        stack
    }

    pub fn destroy(&mut self) {
        let _ = self;
        b3_free(self.memory, self.capacity as usize);
        self.memory = std::ptr::null_mut();
        self.capacity = 0;
        self.index = 0;
        self.allocation = 0;
        self.entry_count = 0;
    }

    pub fn alloc(&mut self, size: i32) -> *mut u8 {
        if self.entry_count as usize == MAX_STACK_ENTRIES {
            b3_assert(false);
            return std::ptr::null_mut();
        }

        let aligned_size = ((size - 1) | (constants::ALIGNMENT as i32 - 1)) + 1;

        let mut entry = StackEntry::default();
        entry.size = aligned_size;
        if self.index + aligned_size > self.capacity {
            entry.data = b3_alloc(aligned_size as usize);
            entry.used_malloc = true;
        } else {
            entry.data = unsafe { self.memory.add(self.index as usize) };
            entry.used_malloc = false;
            self.index += aligned_size;
        }

        self.allocation += aligned_size;
        if self.allocation > self.max_allocation {
            self.max_allocation = self.allocation;
        }

        self.entries[self.entry_count as usize] = entry;
        self.entry_count += 1;
        entry.data
    }

    pub fn free(&mut self, memory: *mut u8) {
        b3_assert(self.entry_count > 0);
        let index = self.entry_count - 1;
        let entry = self.entries[index as usize];
        b3_assert(memory == entry.data);
        if entry.used_malloc {
            b3_free(memory, entry.size as usize);
        } else {
            self.index -= entry.size;
        }
        self.allocation -= entry.size;
        self.entry_count -= 1;
    }

    /// Grow the stack based on usage.
    pub fn grow(&mut self) {
        b3_assert(self.allocation == 0);
        if self.max_allocation > self.capacity {
            b3_free(self.memory, self.capacity as usize);
            self.capacity = self.max_allocation + self.max_allocation / 2;
            self.memory = b3_alloc(self.capacity as usize);
        }
    }
}

#[derive(Clone, Copy)]
pub struct OverflowBlock {
    pub data: *mut u8,
    pub size: i32,
}

impl Default for OverflowBlock {
    fn default() -> Self {
        Self {
            data: std::ptr::null_mut(),
            size: 0,
        }
    }
}

/// Shared, heap-allocated state co-owned by every copy of an [`Arena`].
pub struct ArenaSharedState {
    pub overflows: DynamicArray<OverflowBlock>,
    /// high water mark of the bump pointer this step
    pub max_index: i32,
    /// total bytes in overflow blocks this step
    pub overflow_bytes: i32,
    /// all-time peak of `(max_index + overflow_bytes)`, survives sync
    pub peak_demand: i32,
}

impl Default for ArenaSharedState {
    fn default() -> Self {
        Self {
            overflows: DynamicArray::new(),
            max_index: 0,
            overflow_bytes: 0,
            peak_demand: 0,
        }
    }
}

/// 16-byte alignment for SIMD and typical struct alignment.
const ARENA_ALIGNMENT: i32 = 16;

#[derive(Clone)]
pub struct Arena {
    pub memory: *mut u8,
    pub capacity: i32,
    pub index: i32,
    pub shared: *mut ArenaSharedState,
}

unsafe impl Send for Arena {}

impl Default for Arena {
    fn default() -> Self {
        Self {
            memory: std::ptr::null_mut(),
            capacity: 0,
            index: 0,
            shared: std::ptr::null_mut(),
        }
    }
}

impl Arena {
    pub fn new(capacity: i32) -> Self {
        let c = capacity.max(8);
        let shared = b3_alloc_typed::<ArenaSharedState>(1);
        unsafe {
            shared.write(ArenaSharedState::default());
        }

        let memory = b3_alloc(c as usize);
        Self {
            memory,
            capacity: c,
            index: 0,
            shared,
        }
    }

    pub fn destroy(&mut self) {
        if !self.shared.is_null() {
            unsafe {
                let state = &mut *self.shared;
                for i in 0..state.overflows.count() {
                    let block = state.overflows.get(i);
                    b3_free(block.data, block.size as usize);
                }
                state.overflows.destroy();
                std::ptr::drop_in_place(self.shared);
            }
            b3_free_typed(self.shared, 1);
        }
        b3_free(self.memory, self.capacity as usize);
        *self = Arena::default();
    }

    /// Heap-allocate an overflow block, register it in the shared state, return it.
    fn overflow_alloc(&self, size: i32) -> *mut u8 {
        let shared = self.shared;
        b3_assert(!shared.is_null());
        let data = b3_alloc(size as usize);
        unsafe {
            (*shared).overflows.push(OverflowBlock { data, size });
            (*shared).overflow_bytes += size;
        }
        data
    }

    /// Call between simulation steps. Frees this step's overflow blocks and grows the backing
    /// capacity if last step's demand exceeded it.
    pub fn sync(&mut self) {
        b3_assert(!self.shared.is_null());
        unsafe {
            let shared = &mut *self.shared;
            for i in 0..shared.overflows.count() {
                let block = shared.overflows.get(i);
                b3_free(block.data, block.size as usize);
            }
            shared.overflows.clear();

            let demand = shared.max_index + shared.overflow_bytes;
            if demand > shared.peak_demand {
                shared.peak_demand = demand;
            }
            if demand > self.capacity {
                b3_free(self.memory, self.capacity as usize);
                let new_capacity = demand + demand / 2;
                self.memory = b3_alloc(new_capacity as usize);
                self.capacity = new_capacity;
            }

            self.index = 0;
            shared.max_index = 0;
            shared.overflow_bytes = 0;
        }
    }

    #[inline(always)]
    pub fn bump(&mut self, size: i32) -> *mut u8 {
        if size == 0 {
            return std::ptr::null_mut();
        }

        let aligned = (self.index + (ARENA_ALIGNMENT - 1)) & !(ARENA_ALIGNMENT - 1);

        if aligned + size > self.capacity {
            return self.overflow_alloc(size);
        }

        self.index = aligned + size;
        unsafe {
            b3_assert(!self.shared.is_null());
            if self.index > (*self.shared).max_index {
                (*self.shared).max_index = self.index;
            }
        }
        unsafe { self.memory.add(aligned as usize) }
    }

    /// Typed bump allocation of `count` uninitialized elements.
    #[inline(always)]
    pub fn bump_typed<T>(&mut self, count: i32) -> *mut T {
        self.bump(count * std::mem::size_of::<T>() as i32).cast::<T>()
    }
}

impl Drop for Arena {
    fn drop(&mut self) {
        // The owner is expected to call `destroy`. Free backing memory to avoid leaks in
        // translated code that relies on scope drops.
        if !self.memory.is_null() {
            self.destroy();
        }
    }
}
