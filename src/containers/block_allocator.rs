//! Fixed-size element allocator with stable addresses and an intrusive free list.

use crate::base::core::*;

const BLOCK_EXPONENT: i32 = 8;
const BLOCK_SIZE: i32 = 1 << BLOCK_EXPONENT;
const BLOCK_ALIGNMENT: i32 = 16;

pub struct BlockAllocator {
    blocks: Vec<Box<[u8]>>,
    free_list: Option<usize>,
    element_size: i32,
    next_index: i32,
    allocation_count: i32,
}

impl BlockAllocator {
    pub fn new() -> Self {
        Self {
            blocks: Vec::new(),
            free_list: None,
            element_size: 0,
            next_index: 0,
            allocation_count: 0,
        }
    }

    /// Element must be large enough to hold a pointer. The element size is rounded up to the
    /// block alignment.
    pub fn with_element_size(element_size: i32, initial_count: i32) -> Self {
        b3_assert(element_size >= std::mem::size_of::<usize>() as i32);

        let element_size = ((element_size - 1) | (BLOCK_ALIGNMENT - 1)) + 1;
        let mut allocator = Self {
            blocks: Vec::new(),
            free_list: None,
            element_size,
            next_index: 0,
            allocation_count: 0,
        };

        if initial_count > 0 {
            let count = (initial_count + BLOCK_SIZE - 1) >> BLOCK_EXPONENT;
            allocator.blocks.reserve(count as usize);
            for _ in 0..count {
                allocator.blocks.push(allocator.new_block());
            }
        }
        allocator
    }

    fn new_block(&self) -> Box<[u8]> {
        vec![0u8; (BLOCK_SIZE * self.element_size) as usize].into_boxed_slice()
    }

    pub fn destroy(&mut self) {
        self.blocks.clear();
        self.free_list = None;
        self.next_index = 0;
        self.allocation_count = 0;
    }

    #[inline]
    fn pointer_for_index(&self, index: i32) -> *mut u8 {
        let block_index = (index >> BLOCK_EXPONENT) as usize;
        let block_offset = (index & (BLOCK_SIZE - 1)) as usize;
        unsafe {
            self.blocks[block_index]
                .as_ptr()
                .add(block_offset * self.element_size as usize)
                .cast_mut()
        }
    }

    /// Returns one element of `element_size` contiguous bytes. Address is stable until freed.
    pub fn allocate(&mut self) -> *mut u8 {
        self.allocation_count += 1;

        if let Some(element) = self.free_list {
            let next = unsafe { std::ptr::read_unaligned(self.pointer_for_index(element as i32).cast::<usize>()) };
            self.free_list = if next == usize::MAX { None } else { Some(next) };
            return self.pointer_for_index(element as i32);
        }

        let index = self.next_index;
        self.next_index += 1;

        let required_block_count = (index >> BLOCK_EXPONENT) + 1;
        while required_block_count > self.blocks.len() as i32 {
            self.blocks.push(self.new_block());
        }

        self.pointer_for_index(index)
    }

    pub fn free(&mut self, element: *mut u8) {
        b3_assert(self.allocation_count > 0);
        b3_assert((element as usize) & (BLOCK_ALIGNMENT as usize - 1) == 0);

        self.allocation_count -= 1;

        // Locate the element index to push onto the free list.
        let element_size = self.element_size as usize;
        let block_bytes = (BLOCK_SIZE * self.element_size) as usize;
        let mut found: Option<usize> = None;
        for (block_index, block) in self.blocks.iter().enumerate() {
            let start = block.as_ptr() as usize;
            let address = element as usize;
            if address >= start && address < start + block_bytes {
                let offset = (address - start) / element_size;
                found = Some(block_index * BLOCK_SIZE as usize + offset);
                break;
            }
        }

        let index = found.expect("freed pointer does not belong to BlockAllocator");
        let next = self.free_list.unwrap_or(usize::MAX);
        unsafe {
            std::ptr::write_unaligned(self.pointer_for_index(index as i32).cast::<usize>(), next);
        }
        self.free_list = Some(index);
    }

    pub fn allocation_count(&self) -> i32 {
        self.allocation_count
    }

    pub fn element_size(&self) -> i32 {
        self.element_size
    }
}

impl Default for BlockAllocator {
    fn default() -> Self {
        Self::new()
    }
}
