//! Contact metadata is POD. Variable-size data is owned by the world in stable slabs, so
//! copying a contact handle never retains arrays or makes its manifold buffer shared.

use std::marker::PhantomData;

use crate::base::core::{b3_alloc_typed, b3_assert, b3_free_typed};
use crate::collision::manifold::ClipVertex;
use crate::simulation::mesh_contact::cache::MeshContactState;
use crate::simulation::mesh_contact::reduction::MeshPatchPoint;
use crate::simulation::mesh_contact::selection::MeshManifoldCluster;
use crate::simulation::mesh_contact::triangles::{MeshSurvivor, MeshTriangleManifold};
use crate::types::collision_types::{LocalManifoldPoint, Manifold};

/// The manifold list of one contact. The overwhelmingly common convex case needs no heap
/// allocation: one manifold lives inline, the rest spill to `extra`.
#[derive(Clone, Debug, Default)]
#[repr(C)]
pub struct ContactManifolds {
    pub first: Manifold,
    pub extra: Vec<Manifold>,
    count: usize,
}

impl ContactManifolds {
    pub fn new() -> Self {
        Self::default()
    }

    /// `init(_ manifold: Manifold)`
    pub fn from_manifold(manifold: Manifold) -> Self {
        let mut out = Self::default();
        out.first = manifold;
        out.count = 1;
        out
    }

    /// `init(arrayLiteral elements: Manifold...)`
    pub fn from_elements(elements: &[Manifold]) -> Self {
        let mut out = Self::default();
        out.count = elements.len();
        if out.count == 1 {
            out.first = elements[0];
        } else {
            out.extra = elements.to_vec();
        }
        out
    }

    pub fn start_index(&self) -> usize {
        0
    }

    pub fn end_index(&self) -> usize {
        self.count
    }

    /// `private(set) var count`
    #[inline(always)]
    pub fn count(&self) -> usize {
        self.count
    }

    /// `subscript(_ index: Int) -> Manifold` getter.
    #[inline(always)]
    pub fn get(&self, index: usize) -> Manifold {
        // Swift's precondition also tested `index >= 0`; `usize` makes that unrepresentable.
        b3_assert(index < self.count());
        if self.count == 1 {
            self.first
        } else {
            self.extra[index]
        }
    }

    /// `subscript(_ index: Int) -> Manifold` setter.
    #[inline(always)]
    pub fn set(&mut self, index: usize, value: Manifold) {
        b3_assert(index < self.count);
        if self.count == 1 {
            self.first = value;
        } else {
            self.extra[index] = value;
        }
    }

    pub fn append(&mut self, value: Manifold) {
        if self.count == 0 {
            self.first = value;
        } else if self.count == 1 {
            self.extra = vec![self.first, value];
        } else {
            self.extra.push(value);
        }
        self.count += 1;
    }

    pub fn remove_all(&mut self, keeping_capacity: bool) {
        self.count = 0;
        if keeping_capacity {
            self.extra.clear();
        } else {
            self.extra = Vec::new();
        }
    }

    /// Borrow the stored manifolds as one contiguous buffer.
    #[inline(always)]
    pub fn with_unsafe_mutable_buffer_pointer<R, F>(&mut self, body: F) -> R
    where
        F: FnOnce(&mut [Manifold]) -> R,
    {
        // SAFETY: with a single manifold the buffer is the inline `first` field; the borrow is
        // exclusive (`&mut self`) and one element long, so it cannot alias `extra`.
        let buffer = unsafe {
            if self.count == 1 {
                std::slice::from_raw_parts_mut(&mut self.first, 1)
            } else {
                self.extra.as_mut_slice()
            }
        };
        body(buffer)
    }

    /// Borrow the stored manifolds as one contiguous read-only buffer.
    #[inline(always)]
    pub fn with_unsafe_buffer_pointer<R, F>(&self, body: F) -> R
    where
        F: FnOnce(&[Manifold]) -> R,
    {
        // SAFETY: one element of `first` when the count is 1; the slice is read-only and `&self`
        // keeps it exclusive to this call.
        let buffer = unsafe {
            if self.count == 1 {
                std::slice::from_raw_parts(&self.first, 1)
            } else {
                self.extra.as_slice()
            }
        };
        body(buffer)
    }
}

impl PartialEq for ContactManifolds {
    fn eq(&self, other: &Self) -> bool {
        self.count == other.count
            && (0..self.count).all(|index| self.get(index) == other.get(index))
    }
}

/// Everything a contact owns beyond its POD metadata.
#[derive(Clone, Default)]
#[repr(C)]
pub struct ContactPayload {
    pub manifolds: ContactManifolds,
    pub mesh_contact: MeshContactState,
}

/// Reusable scratch buffers for one worker's mesh narrow phase.
#[derive(Clone, Debug, Default)]
pub struct MeshContactScratch {
    pub survivors: Vec<MeshSurvivor>,
    pub manifolds: Vec<MeshTriangleManifold>,
    pub points: Vec<LocalManifoldPoint>,

    /// Per-worker diagnostics. Keeping these off the World objects avoids an exclusivity check
    /// on every hull pair and contact.
    pub sat_calls: i32,
    pub sat_cache_hits: i32,
    pub recycled_contacts: i32,

    /// Reused SAT and face-clipping slabs. `withUnsafeTemporaryAllocation` falls back to the heap
    /// above 1 KB, and these are 13 KB and 8 KB, so keeping them here removes a malloc/free from
    /// every hull pair.
    pub sat_scratch: Vec<u8>,
    pub face_scratch: Vec<u8>,

    /// Reused mesh-manifold selection and clustering storage.
    pub selected_manifolds: Vec<MeshTriangleManifold>,
    pub clusters: Vec<MeshManifoldCluster>,
    pub consumed_manifolds: Vec<bool>,
    pub projected_points: Vec<MeshPatchPoint>,
}

pub const SAT_SCRATCH_BYTE_COUNT: usize = 25 * 132 * 4;
/// `MemoryLayout<T>.stride` equals `size_of::<T>()` for these `#[repr(C)]` records.
pub const FACE_SCRATCH_BYTE_COUNT: usize =
    2 * 64 * core::mem::size_of::<ClipVertex>() + 64 * core::mem::size_of::<LocalManifoldPoint>();

/// Slabs never move. Contact ids reuse the same initialized payload slot, while the pool
/// releases every non-POD field on world destruction, including slots of destroyed contacts.
pub struct ContactPayloadPool {
    blocks: Vec<*mut ContactPayload>,
}

/// `private static let blockSize`
const BLOCK_SIZE: usize = 256;

impl ContactPayloadPool {
    pub fn new() -> Self {
        Self { blocks: Vec::new() }
    }

    /// Return the payload slot for `index`, growing the slab list as needed.
    pub fn initialize(&mut self, index: usize) -> *mut ContactPayload {
        let block_index = index / BLOCK_SIZE;
        while self.blocks.len() <= block_index {
            // SAFETY: `b3_alloc_typed` returns freshly allocated memory aligned for
            // `ContactPayload` and holds exactly `BLOCK_SIZE` elements; every element is written
            // before any is read, so the block is fully initialized once the loop finishes.
            let block = unsafe { b3_alloc_typed::<ContactPayload>(BLOCK_SIZE as i32) };
            unsafe {
                for offset in 0..BLOCK_SIZE {
                    block.add(offset).write(ContactPayload::default());
                }
            }
            self.blocks.push(block);
        }
        let result = self.blocks[block_index].wrapping_add(index % BLOCK_SIZE);
        // Assigning through the slot (not `ptr::write`) releases the previous payload's non-POD
        // fields, mirroring Swift's `pointee =` on already-initialized memory.
        // SAFETY: the slot is inside the block created above and is initialized.
        unsafe {
            *result = ContactPayload::default();
        }
        result
    }
}

impl Default for ContactPayloadPool {
    fn default() -> Self {
        Self::new()
    }
}

// SAFETY: raw-pointer blocks make the pool non-`Send` by default. A pool is owned by exactly one
// world, which hands out payload pointers; the world itself documents the single-owner discipline
// for its pools, so moving the pool together with its world is sound.
unsafe impl Send for ContactPayloadPool {}

impl Drop for ContactPayloadPool {
    fn drop(&mut self) {
        for block in self.blocks.drain(..) {
            // SAFETY: each block was allocated and fully initialized by `initialize`; dropping
            // every element releases the manifolds and mesh state it owns before the memory is
            // returned, including slots of contacts that were destroyed earlier.
            unsafe {
                for offset in 0..BLOCK_SIZE {
                    std::ptr::drop_in_place(block.add(offset));
                }
                b3_free_typed(block, BLOCK_SIZE as i32);
            }
        }
    }
}

/// A borrowed view carries one pointer, so iteration and element access never copy the
/// inline manifold or allocate a Swift coroutine frame. Its owner is [`ContactPayloadPool`].
///
// SAFETY: `payload` addresses a slot of a `ContactPayloadPool` block. Blocks are allocated once
// and never reallocated or moved, so the raw pointer stays valid for the whole `'p` borrow of the
// pool; `PhantomData` records that borrow so the view cannot outlive its owner. Mutating through
// `&self` mirrors Swift's non-mutating `unsafeMutableAddress` accessors: the view is a window into
// pool storage, not a value that owns it.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct ContactManifoldView<'p> {
    pub payload: *mut ContactPayload,
    borrowed: PhantomData<&'p mut ContactPayload>,
}

impl<'p> ContactManifoldView<'p> {
    pub fn new(payload: *mut ContactPayload) -> Self {
        Self {
            payload,
            borrowed: PhantomData,
        }
    }

    pub fn start_index(&self) -> usize {
        0
    }

    #[inline(always)]
    pub fn end_index(&self) -> usize {
        self.count()
    }

    #[inline(always)]
    pub fn count(&self) -> usize {
        // SAFETY: `payload` addresses an initialized pool slot.
        unsafe { (*self.payload).manifolds.count() }
    }

    #[inline(always)]
    fn payload_ref(&self) -> &'p ContactPayload {
        // SAFETY: see the type's SAFETY note; the slot lives for `'p`.
        unsafe { &*self.payload }
    }

    #[inline(always)]
    fn payload_mut(&self) -> &'p mut ContactPayload {
        // SAFETY: the pool owns this slot exclusively and hands out one view at a time.
        unsafe { &mut *self.payload }
    }

    /// Address of the inline first manifold, computed from the payload layout exactly as the
    /// Swift view did so it never has to borrow the manifold list itself.
    #[inline(always)]
    fn first_pointer(&self) -> *mut Manifold {
        let offset = core::mem::offset_of!(ContactPayload, manifolds)
            + core::mem::offset_of!(ContactManifolds, first);
        // SAFETY: `payload` is in bounds of a `ContactPayload`, so advancing it by the offset of
        // `manifolds.first` lands on a live `Manifold`.
        unsafe { (self.payload as *mut u8).add(offset).cast::<Manifold>() }
    }

    #[inline(always)]
    pub fn get(&self, index: usize) -> &'p Manifold {
        b3_assert(index < self.count());
        if self.count() == 1 {
            // SAFETY: the inline manifold is one element wide and the count just checked it.
            unsafe { &*self.first_pointer() }
        } else {
            &self.payload_ref().manifolds.extra[index]
        }
    }

    #[inline(always)]
    pub fn get_mut(&self, index: usize) -> &'p mut Manifold {
        b3_assert(index < self.count());
        if self.count() == 1 {
            // SAFETY: see `get`; the pool slot is the sole owner of the manifold storage.
            unsafe { &mut *self.first_pointer() }
        } else {
            &mut self.payload_mut().manifolds.extra[index]
        }
    }

    pub fn remove_all(&self, keeping_capacity: bool) {
        self.payload_mut().manifolds.remove_all(keeping_capacity);
    }

    pub fn replace_with(&self, storage: ContactManifolds) {
        self.payload_mut().manifolds = storage;
    }

    #[inline(always)]
    pub fn with_unsafe_mutable_buffer_pointer<R, F>(&self, body: F) -> R
    where
        F: FnOnce(&mut [Manifold]) -> R,
    {
        // SAFETY: exactly one of the two storages backs the view, chosen by the same count test
        // the Swift version used; the buffer is a window into the pool slot for the call only.
        let buffer = unsafe {
            if self.count() == 1 {
                std::slice::from_raw_parts_mut(self.first_pointer(), 1)
            } else {
                self.payload_mut().manifolds.extra.as_mut_slice()
            }
        };
        body(buffer)
    }

    #[inline(always)]
    pub fn buffer(&self) -> &'p [Manifold] {
        if self.count() == 1 {
            // SAFETY: one initialized manifold at `first_pointer`.
            unsafe { std::slice::from_raw_parts(self.first_pointer(), 1) }
        } else {
            self.payload_ref().manifolds.extra.as_slice()
        }
    }
}

impl PartialEq for ContactManifoldView<'_> {
    fn eq(&self, other: &Self) -> bool {
        self.count() == other.count()
            && (0..self.count()).all(|index| self.get(index) == other.get(index))
    }
}

impl crate::simulation::world_runtime::Contact {
    /// `var manifolds: ContactManifoldView`
    pub fn manifolds(&self) -> ContactManifoldView<'_> {
        // SAFETY: the pool keeps the payload alive for as long as the contact does; a contact
        // without a payload has already been destroyed, which traps here exactly as Swift's
        // force unwrap of `payload` would.
        match self.payload {
            Some(payload) => ContactManifoldView::new(payload),
            None => crate::base::core::internal_assert("contact payload is null", file!(), line!()),
        }
    }

    /// `var meshContact: MeshContactState`
    pub fn mesh_contact(&self) -> &crate::simulation::mesh_contact::cache::MeshContactState {
        // SAFETY: the offset lands on the `meshContact` field of an initialized pool slot, and
        // the slot is never moved while this contact exists.
        let offset = core::mem::offset_of!(ContactPayload, mesh_contact);
        match self.payload {
            Some(payload) => unsafe {
                &*(payload as *mut u8).add(offset).cast::<
                    crate::simulation::mesh_contact::cache::MeshContactState,
                >()
            },
            None => crate::base::core::internal_assert("contact payload is null", file!(), line!()),
        }
    }

    /// Mutable access to the mesh contact state stored in the payload slab. Swift's accessor was
    /// `nonmutating`; Rust takes `&mut self` because it hands out an exclusive reference.
    pub fn mesh_contact_mut(
        &mut self,
    ) -> &mut crate::simulation::mesh_contact::cache::MeshContactState {
        let offset = core::mem::offset_of!(ContactPayload, mesh_contact);
        // SAFETY: see `mesh_contact`; `&mut self` makes this the only live borrow of the slot.
        match self.payload {
            Some(payload) => unsafe {
                &mut *(payload as *mut u8).add(offset).cast::<
                    crate::simulation::mesh_contact::cache::MeshContactState,
                >()
            },
            None => crate::base::core::internal_assert("contact payload is null", file!(), line!()),
        }
    }

    /// Public query views remain valid until the next world mutation.
    pub fn manifold_buffer(&self) -> &[Manifold] {
        match self.payload {
            Some(payload) => ContactManifoldView::new(payload).buffer(),
            None => &[],
        }
    }
}
