//! Contact storage view.
//!
//! The collide pass runs in parallel, so contact writes must not go through the World's
//! `contacts` array: that would copy-on-write the whole buffer per write and race across workers.
//! Every narrow-phase routine therefore takes a `ContactStore`, which is either a plain reference
//! to the array (serial) or a mutable window over it (parallel). Writes land in place either way.

use crate::simulation::world_runtime::Contact;

/// Which of the two storages a store points at. Swift picked between them with
/// `buffer.count > 0`; the variant carries the same information without raw pointers.
enum Storage<'a> {
    /// Wrap of the world's contact array, for serial access.
    Array(&'a mut Vec<Contact>),
    /// Wrap of a window of that array, for parallel access. Only elements inside the window may
    /// be written.
    Buffer(&'a mut [Contact]),
}

/// A borrowed handle on contact storage that never copies the array.
pub struct ContactStore<'a> {
    storage: Storage<'a>,
}

impl<'a> ContactStore<'a> {
    /// Wrap the world's array for serial access.
    ///
    /// Swift's `init(_ array: inout ContiguousArray<Contact>)`; the Rust equivalent of
    /// `ContiguousArray` is `Vec`, and an exclusive borrow replaces the borrowed pointer.
    #[inline(always)]
    pub fn from_array(array: &'a mut Vec<Contact>) -> Self {
        Self {
            storage: Storage::Array(array),
        }
    }

    /// Wrap a buffer for parallel access. Only elements inside the buffer may be written.
    #[inline(always)]
    pub fn from_buffer(buffer: &'a mut [Contact]) -> Self {
        Self {
            storage: Storage::Buffer(buffer),
        }
    }

    #[inline(always)]
    pub fn get(&self, id: i32) -> Contact {
        let index = id as usize;
        match &self.storage {
            Storage::Buffer(buffer) => buffer[index].clone(),
            Storage::Array(array) => array[index].clone(),
        }
    }

    /// Writes land in place, in the storage the store was built from. Swift's setter was
    /// non-mutating because it wrote through a raw buffer pointer.
    #[inline(always)]
    pub fn set(&mut self, id: i32, contact: Contact) {
        let index = id as usize;
        match &mut self.storage {
            Storage::Buffer(buffer) => buffer[index] = contact,
            Storage::Array(array) => array[index] = contact,
        }
    }

    /// Borrow one contact for in-place mutation without copying it.
    ///
    /// `Contact` carries three reference-counted fields, so a get/modify/set round trip retains and
    /// releases several arrays per contact. This hands out the stored element directly.
    #[inline(always)]
    pub fn with_contact<R, F>(&mut self, id: i32, body: F) -> R
    where
        F: FnOnce(&mut Contact) -> R,
    {
        let index = id as usize;
        match &mut self.storage {
            Storage::Buffer(buffer) => body(&mut buffer[index]),
            Storage::Array(array) => body(&mut array[index]),
        }
    }

    #[inline(always)]
    pub fn count(&self) -> usize {
        match &self.storage {
            Storage::Buffer(buffer) => buffer.len(),
            Storage::Array(array) => array.len(),
        }
    }
}
