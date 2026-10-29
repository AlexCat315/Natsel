//! Platform primitives: timing, threads, atomics, spin hints, and prefetch.

use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, AtomicI64, AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

/// Get the absolute number of system ticks. The value is platform specific.
pub fn get_ticks() -> u64 {
    use std::sync::OnceLock;
    static ORIGIN: OnceLock<Instant> = OnceLock::new();
    let origin = ORIGIN.get_or_init(Instant::now);
    origin.elapsed().as_nanos() as u64
}

/// Milliseconds passed from an initial tick value.
pub fn get_milliseconds(ticks: u64) -> f32 {
    let now = get_ticks();
    (now.wrapping_sub(ticks) as f64 * 1e-6) as f32
}

/// Milliseconds passed from an initial tick value and reset the tick value.
pub fn get_milliseconds_and_reset(ticks: &mut u64) -> f32 {
    let now = get_ticks();
    let ms = (now.wrapping_sub(*ticks) as f64 * 1e-6) as f32;
    *ticks = now;
    ms
}

/// Yield to be used in a busy loop.
pub fn yield_thread() {
    std::thread::yield_now();
}

/// Sleep the current thread for a number of milliseconds.
pub fn sleep_thread(milliseconds: i32) {
    if milliseconds > 0 {
        std::thread::sleep(Duration::from_millis(milliseconds as u64));
    }
}

/// Number of performance cores. Falls back to available parallelism.
pub fn performance_core_count() -> usize {
    std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(1)
        .max(1)
}

/// Processor hint for a busy wait.
#[inline(always)]
pub fn spin_pause() {
    std::hint::spin_loop();
}

/// Read prefetch with high temporal locality.
#[inline(always)]
pub fn b3_prefetch<T>(address: *const T) {
    #[cfg(target_arch = "aarch64")]
    unsafe {
        core::arch::asm!("prfm pldl1keep, [{0}]", in(reg) address, options(nostack, preserves_flags));
    }
    #[cfg(target_arch = "x86_64")]
    unsafe {
        core::arch::asm!("prefetcht0 [{0}]", in(reg) address, options(nostack, preserves_flags));
    }
    #[cfg(not(any(target_arch = "aarch64", target_arch = "x86_64")))]
    {
        let _ = address;
    }
}

/// A simple mutex matching the engine's primitive.
#[derive(Debug)]
pub struct B3Mutex<T> {
    inner: Mutex<T>,
}

impl<T> B3Mutex<T> {
    pub fn new(value: T) -> Self {
        Self {
            inner: Mutex::new(value),
        }
    }

    pub fn lock(&self) -> std::sync::MutexGuard<'_, T> {
        self.inner.lock().unwrap_or_else(|e| e.into_inner())
    }
}

/// A simple counting semaphore.
#[derive(Debug)]
pub struct B3Semaphore {
    state: Mutex<usize>,
    cond: Condvar,
}

impl B3Semaphore {
    pub fn new(count: usize) -> Self {
        Self {
            state: Mutex::new(count),
            cond: Condvar::new(),
        }
    }

    pub fn wait(&self) {
        let mut count = self.state.lock().unwrap_or_else(|e| e.into_inner());
        while *count == 0 {
            count = self.cond.wait(count).unwrap_or_else(|e| e.into_inner());
        }
        *count -= 1;
    }

    pub fn signal(&self) {
        let mut count = self.state.lock().unwrap_or_else(|e| e.into_inner());
        *count += 1;
        self.cond.notify_one();
    }
}

/// A joinable worker thread.
pub struct B3Thread {
    handle: Option<std::thread::JoinHandle<()>>,
}

impl B3Thread {
    /// Name may be `None`; otherwise it is used for the OS thread name (truncated to 15 bytes).
    pub fn new<F>(name: Option<&str>, function: F) -> Self
    where
        F: FnOnce() + Send + 'static,
    {
        let mut builder = std::thread::Builder::new();
        if let Some(name) = name {
            let short: String = name.chars().take(15).collect();
            builder = builder.name(short);
        }
        let handle = builder
            .spawn(function)
            .expect("Could not create physics worker thread");
        Self {
            handle: Some(handle),
        }
    }

    pub fn join(&mut self) {
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

impl Drop for B3Thread {
    fn drop(&mut self) {
        self.join();
    }
}

/// Plain storage for an atomic integer.
pub type AtomicInt = AtomicI32;
pub type AtomicU32Value = AtomicU32;
pub type AtomicI64Value = AtomicI64;
pub type AtomicU64Value = AtomicU64;
pub type AtomicBoolValue = AtomicBool;

#[inline(always)]
pub fn atomic_store_int(a: &AtomicI32, value: i32) {
    a.store(value, Ordering::SeqCst);
}

#[inline(always)]
pub fn atomic_load_int(a: &AtomicI32) -> i32 {
    a.load(Ordering::SeqCst)
}

#[inline(always)]
pub fn atomic_fetch_add_int(a: &AtomicI32, increment: i32) -> i32 {
    a.fetch_add(increment, Ordering::SeqCst)
}

#[inline(always)]
pub fn atomic_compare_exchange_int(a: &AtomicI32, expected: i32, desired: i32) -> bool {
    a.compare_exchange(expected, desired, Ordering::SeqCst, Ordering::SeqCst)
        .is_ok()
}

#[inline(always)]
pub fn atomic_store_u32(a: &AtomicU32, value: u32) {
    a.store(value, Ordering::SeqCst);
}

#[inline(always)]
pub fn atomic_load_u32(a: &AtomicU32) -> u32 {
    a.load(Ordering::SeqCst)
}

/// Relaxed load on a plain word.
#[inline(always)]
pub fn atomic_load_u32_raw(a: &AtomicU32) -> u32 {
    a.load(Ordering::Relaxed)
}

#[inline(always)]
pub fn atomic_fetch_or_u32(a: &AtomicU32, mask: u32) -> u32 {
    a.fetch_or(mask, Ordering::SeqCst)
}

/// Small wrapper for state shared between threads that is not `Sync` by construction.
///
/// The engine only shares immutable data plus disjoint mutable regions. Callers are responsible
/// for that invariant.
pub struct UnsafeSend<T>(pub T);

unsafe impl<T> Send for UnsafeSend<T> {}
unsafe impl<T> Sync for UnsafeSend<T> {}

/// Shared handle used by the worker pool.
pub type Shared<T> = Arc<T>;
