//! Version, assertions, logging, and memory helpers.

use std::alloc::{alloc, dealloc, Layout};
use std::sync::atomic::{AtomicI64, Ordering};
use std::sync::{Mutex, OnceLock};

use crate::base::constants;

/// Version numbering scheme.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Version {
    /// Significant changes
    pub major: i32,
    /// Incremental changes
    pub minor: i32,
    /// Bug fixes
    pub revision: i32,
}

/// Get the current version of the engine.
pub fn get_version() -> Version {
    Version {
        major: 0,
        minor: 2,
        revision: 0,
    }
}

/// World positions use `f64`; local geometry, velocities, and constraint solving use `f32`.
#[inline]
pub fn is_double_precision() -> bool {
    true
}

/// The maximum manifold point count the library was built with.
#[inline]
pub fn get_max_manifold_points() -> i32 {
    constants::MAX_MANIFOLD_POINTS
}

// MARK: - Assertions

pub type AssertHandler = Box<dyn Fn(&str, &str, u32) -> bool + Send + Sync>;
pub type LogHandler = Box<dyn Fn(&str) + Send + Sync>;

fn assert_handler_cell() -> &'static Mutex<AssertHandler> {
    static CELL: OnceLock<Mutex<AssertHandler>> = OnceLock::new();
    CELL.get_or_init(|| {
        Mutex::new(Box::new(|condition: &str, file: &str, line: u32| {
            eprintln!("NATSEL ASSERTION: {condition}, {file}, line {line}");
            true
        }))
    })
}

fn log_handler_cell() -> &'static Mutex<LogHandler> {
    static CELL: OnceLock<Mutex<LogHandler>> = OnceLock::new();
    CELL.get_or_init(|| Mutex::new(Box::new(|message: &str| {
        println!("Natsel: {message}");
    })))
}

/// Override the default assert callback.
pub fn set_assert_handler(handler: AssertHandler) {
    *assert_handler_cell().lock().unwrap_or_else(|e| e.into_inner()) = handler;
}

/// Override the default logging callback.
pub fn set_log_handler(handler: LogHandler) {
    *log_handler_cell().lock().unwrap_or_else(|e| e.into_inner()) = handler;
}

pub fn internal_assert(condition: &str, file: &str, line: u32) -> ! {
    let should_break = (assert_handler_cell().lock().unwrap_or_else(|e| e.into_inner()))(
        condition, file, line,
    );
    if should_break {
        panic!("assertion failed: {condition} ({file}:{line})");
    }
    panic!("assertion failed: {condition} ({file}:{line})");
}

/// Assert that a condition is true. Active in debug builds only.
#[inline(always)]
pub fn b3_assert(condition: bool) {
    #[cfg(debug_assertions)]
    if !condition {
        internal_assert("assertion", file!(), line!());
    }
    #[cfg(not(debug_assertions))]
    {
        let _ = condition;
    }
}

/// Assert with a message.
#[inline(always)]
pub fn b3_assert_msg(condition: bool, message: &str) {
    #[cfg(debug_assertions)]
    if !condition {
        internal_assert(message, file!(), line!());
    }
    #[cfg(not(debug_assertions))]
    {
        let _ = condition;
        let _ = message;
    }
}

/// Floating point tolerance checks. Only active when built with the `validate` feature.
#[inline(always)]
pub fn b3_validate(condition: bool) {
    #[cfg(all(debug_assertions, feature = "validate"))]
    if !condition {
        internal_assert("validation", file!(), line!());
    }
    #[cfg(not(all(debug_assertions, feature = "validate")))]
    {
        let _ = condition;
    }
}

#[inline(always)]
pub fn b3_validate_msg(condition: bool, message: &str) {
    #[cfg(all(debug_assertions, feature = "validate"))]
    if !condition {
        internal_assert(message, file!(), line!());
    }
    #[cfg(not(all(debug_assertions, feature = "validate")))]
    {
        let _ = condition;
        let _ = message;
    }
}

/// Whether internal validation is enabled.
pub const B3_ENABLE_VALIDATION: bool = cfg!(all(debug_assertions, feature = "validate"));

pub fn b3_log(message: &str) {
    (log_handler_cell().lock().unwrap_or_else(|e| e.into_inner()))(message);
}

// MARK: - Memory

pub type AllocHandler = Box<dyn Fn(usize, usize) -> *mut u8 + Send + Sync>;
pub type FreeHandler = Box<dyn Fn(*mut u8, usize) + Send + Sync>;

fn alloc_handler_cell() -> &'static Mutex<Option<AllocHandler>> {
    static CELL: OnceLock<Mutex<Option<AllocHandler>>> = OnceLock::new();
    CELL.get_or_init(|| Mutex::new(None))
}

fn free_handler_cell() -> &'static Mutex<Option<FreeHandler>> {
    static CELL: OnceLock<Mutex<Option<FreeHandler>>> = OnceLock::new();
    CELL.get_or_init(|| Mutex::new(None))
}

static ALLOCATED_BYTE_COUNT: AtomicI64 = AtomicI64::new(0);

/// This allows the user to override the allocation functions. These should be set during
/// application startup.
pub fn set_allocator(alloc_handler: Option<AllocHandler>, free_handler: Option<FreeHandler>) {
    *alloc_handler_cell().lock().unwrap_or_else(|e| e.into_inner()) = alloc_handler;
    *free_handler_cell().lock().unwrap_or_else(|e| e.into_inner()) = free_handler;
}

/// Total bytes allocated by the engine.
pub fn get_byte_count() -> i64 {
    ALLOCATED_BYTE_COUNT.load(Ordering::SeqCst)
}

fn internal_alloc(size: usize) -> *mut u8 {
    if size == 0 {
        return std::ptr::null_mut();
    }

    ALLOCATED_BYTE_COUNT.fetch_add(size as i64, Ordering::SeqCst);

    let aligned_size = ((size - 1) | (constants::ALIGNMENT - 1)) + 1;

    if let Some(handler) = &*alloc_handler_cell().lock().unwrap_or_else(|e| e.into_inner()) {
        let ptr = handler(aligned_size, constants::ALIGNMENT);
        b3_assert(!ptr.is_null());
        b3_assert((ptr as usize) & (constants::ALIGNMENT - 1) == 0);
        return ptr;
    }

    let layout = Layout::from_size_align(aligned_size, constants::ALIGNMENT).unwrap();
    unsafe { alloc(layout) }
}

/// Allocate raw bytes with the engine alignment.
pub fn b3_alloc(size: usize) -> *mut u8 {
    internal_alloc(size)
}

/// Free raw bytes previously returned by [`b3_alloc`].
pub fn b3_free(memory: *mut u8, size: usize) {
    if memory.is_null() {
        return;
    }

    if let Some(handler) = &*free_handler_cell().lock().unwrap_or_else(|e| e.into_inner()) {
        let aligned_size = if size > 0 {
            ((size - 1) | (constants::ALIGNMENT - 1)) + 1
        } else {
            0
        };
        handler(memory, aligned_size);
    } else {
        let aligned_size = if size > 0 {
            ((size - 1) | (constants::ALIGNMENT - 1)) + 1
        } else {
            0
        };
        let layout = Layout::from_size_align(aligned_size, constants::ALIGNMENT).unwrap();
        unsafe { dealloc(memory, layout) };
    }

    ALLOCATED_BYTE_COUNT.fetch_sub(size as i64, Ordering::SeqCst);
}

/// Allocate zeroed raw bytes.
pub fn b3_alloc_zero(size: usize) -> *mut u8 {
    let memory = b3_alloc(size);
    if !memory.is_null() {
        unsafe { std::ptr::write_bytes(memory, 0, size) };
    }
    memory
}

/// Grow an allocation by copying the old contents.
pub fn b3_grow_alloc(old_memory: *mut u8, old_size: usize, new_size: usize) -> *mut u8 {
    b3_assert(new_size > old_size);
    let new_memory = b3_alloc(new_size);
    if old_size > 0 && !old_memory.is_null() && !new_memory.is_null() {
        unsafe {
            std::ptr::copy_nonoverlapping(old_memory, new_memory, old_size);
        }
        b3_free(old_memory, old_size);
    }
    new_memory
}

/// Grow an allocation and zero the new region.
pub fn b3_grow_alloc_zeroed(
    old_memory: *mut u8,
    old_size: usize,
    new_size: usize,
) -> *mut u8 {
    let new_memory = b3_grow_alloc(old_memory, old_size, new_size);
    if !new_memory.is_null() {
        unsafe { std::ptr::write_bytes(new_memory.add(old_size), 0, new_size - old_size) };
    }
    new_memory
}

/// Typed allocation of `count` uninitialized elements.
#[inline(always)]
pub fn b3_alloc_typed<T>(count: i32) -> *mut T {
    let stride = std::mem::size_of::<T>();
    let raw = b3_alloc(stride * count as usize);
    raw.cast::<T>()
}

/// Typed free of `count` elements.
#[inline(always)]
pub fn b3_free_typed<T>(memory: *mut T, count: i32) {
    b3_free(memory.cast::<u8>(), std::mem::size_of::<T>() * count as usize);
}

/// Typed growth of a buffer from `old_count` to `new_count` elements.
#[inline(always)]
pub fn b3_grow_typed<T>(memory: *mut T, old_count: i32, new_count: i32) -> *mut T {
    let stride = std::mem::size_of::<T>();
    b3_grow_alloc(
        memory.cast::<u8>(),
        old_count as usize * stride,
        new_count as usize * stride,
    )
    .cast::<T>()
}

/// Typed zeroed growth of a buffer from `old_count` to `new_count` elements.
#[inline(always)]
pub fn b3_grow_zero_typed<T>(memory: *mut T, old_count: i32, new_count: i32) -> *mut T {
    let stride = std::mem::size_of::<T>();
    b3_grow_alloc_zeroed(
        memory.cast::<u8>(),
        old_count as usize * stride,
        new_count as usize * stride,
    )
    .cast::<T>()
}
