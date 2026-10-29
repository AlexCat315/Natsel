//! Shared lifetime for geometry allocations and their sliced views.

use core::ptr;
use std::sync::Arc;

use crate::base::core::{b3_alloc_zero, b3_free};

struct GeometryAllocation {
    base: *mut u8,
    byte_count: usize,
}

unsafe impl Send for GeometryAllocation {}
unsafe impl Sync for GeometryAllocation {}

impl Drop for GeometryAllocation {
    fn drop(&mut self) {
        if !self.base.is_null() {
            b3_free(self.base, self.byte_count);
            self.base = ptr::null_mut();
        }
    }
}

/// A borrowed slice of a shared geometry allocation.
#[derive(Clone)]
pub struct GeometryStorage {
    allocation: Arc<GeometryAllocation>,
    offset: usize,
    byte_count: usize,
}

impl GeometryStorage {
    /// Allocate zeroed storage and take ownership of it.
    pub fn new(byte_count: usize) -> Option<Self> {
        let base = b3_alloc_zero(byte_count);
        if base.is_null() {
            return None;
        }
        Some(Self {
            allocation: Arc::new(GeometryAllocation { base, byte_count }),
            offset: 0,
            byte_count,
        })
    }

    /// Take ownership of an existing allocation.
    ///
    /// # Safety
    /// `base` must come from `b3_alloc`/`b3_alloc_zero` with matching `byte_count`.
    pub unsafe fn from_raw(base: *mut u8, byte_count: usize) -> Self {
        Self {
            allocation: Arc::new(GeometryAllocation { base, byte_count }),
            offset: 0,
            byte_count,
        }
    }

    fn with_offset(allocation: Arc<GeometryAllocation>, offset: usize, byte_count: usize) -> Self {
        Self {
            allocation,
            offset,
            byte_count,
        }
    }

    pub fn slice(&self, offset: usize, byte_count: usize) -> Self {
        assert!(offset + byte_count <= self.byte_count);
        Self::with_offset(self.allocation.clone(), self.offset + offset, byte_count)
    }

    pub fn base(&self) -> *mut u8 {
        unsafe { self.allocation.base.add(self.offset) }
    }

    pub fn byte_count(&self) -> usize {
        self.byte_count
    }

    /// Typed view of the whole storage.
    ///
    /// # Safety
    /// `T` must be layout-compatible with the stored bytes.
    pub unsafe fn as_ptr<T>(&self) -> *mut T {
        self.base() as *mut T
    }

    /// View of the storage starting at a byte offset.
    ///
    /// # Safety
    /// The offset and type must address initialized data inside the allocation.
    pub unsafe fn offset_ptr<T>(&self, offset: usize) -> *mut T {
        self.base().add(offset) as *mut T
    }

    /// Typed slice view.
    ///
    /// # Safety
    /// `offset`/`count` must be inside the allocation.
    pub unsafe fn slice_ref<T>(&self, offset: usize, count: usize) -> &'static [T] {
        std::slice::from_raw_parts(self.base().add(offset) as *const T, count)
    }

    /// Mutable typed slice view.
    ///
    /// # Safety
    /// `offset`/`count` must be inside the allocation and no other references may alias.
    pub unsafe fn slice_mut<T>(&self, offset: usize, count: usize) -> &'static mut [T] {
        std::slice::from_raw_parts_mut(self.base().add(offset) as *mut T, count)
    }
}
