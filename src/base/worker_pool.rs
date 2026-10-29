//! A pool of persistent worker threads for fine grained parallel stages.
//!
//! Between `begin` and `end` the workers spin on a stage counter so a new stage starts within
//! microseconds; outside they sleep on a semaphore. A stage is a number of blocks. The main
//! thread publishes a stage, works on it too, and waits until every block is done.

use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

use crate::base::platform::{spin_pause, yield_thread, B3Semaphore, B3Thread};

/// The plain data describing one stage. Copied by value.
#[derive(Clone, Copy, Debug, Default)]
pub struct WorkerStage {
    pub kind: i32,
    pub color: i32,
    pub block_count: i32,
    pub block_size: i32,
    pub item_count: i32,
    pub flags: i32,
}

type Handler = Arc<dyn Fn(WorkerStage, usize, usize) + Send + Sync>;

/// Wrap a closure so it can be shared with worker threads. The closure is responsible for any
/// unsafe sharing it performs.
pub struct SendFn<F>(pub F);

unsafe impl<F> Send for SendFn<F> {}
unsafe impl<F> Sync for SendFn<F> {}

struct WorkerPoolShared {
    wake: B3Semaphore,
    running: AtomicBool,
    shutdown: AtomicBool,
    awake: AtomicI32,
    completed: AtomicUsize,
    next_block: AtomicUsize,
    current: AtomicU32,
    stage: Mutex<WorkerStage>,
    handler: Mutex<Option<Handler>>,
    thread_count: usize,
}

impl WorkerPoolShared {
    const HOT_SPINS: usize = 1 << 16;
    const MAX_BLOCKS: usize = 256;

    fn new(thread_count: usize) -> Self {
        Self {
            wake: B3Semaphore::new(0),
            running: AtomicBool::new(false),
            shutdown: AtomicBool::new(false),
            awake: AtomicI32::new(0),
            completed: AtomicUsize::new(0),
            next_block: AtomicUsize::new(0),
            current: AtomicU32::new(0),
            stage: Mutex::new(WorkerStage::default()),
            handler: Mutex::new(None),
            thread_count,
        }
    }

    /// Claim and run blocks of the published stage until none are left.
    fn work(&self, handler: &Handler, stage: WorkerStage, thread: usize) {
        let count = stage.block_count as usize;
        loop {
            let block = self.next_block.fetch_add(1, Ordering::AcqRel);
            if block >= count {
                break;
            }
            handler(stage, block, thread);
            self.completed.fetch_add(1, Ordering::AcqRel);
        }
    }

    fn worker_main(self: &Arc<Self>, thread: usize) {
        loop {
            self.wake.wait();
            if self.shutdown.load(Ordering::SeqCst) {
                return;
            }
            self.awake.fetch_add(1, Ordering::SeqCst);

            let mut seen = self.current.load(Ordering::Acquire);
            let mut spins = 0usize;
            while self.running.load(Ordering::Acquire) {
                let current = self.current.load(Ordering::Acquire);
                if current != seen {
                    seen = current;
                    let stage = *self.stage.lock().unwrap_or_else(|e| e.into_inner());
                    let handler = self
                        .handler
                        .lock()
                        .unwrap_or_else(|e| e.into_inner())
                        .clone();
                    if let Some(handler) = handler {
                        self.work(&handler, stage, thread);
                    }
                    spins = 0;
                    continue;
                }
                spins += 1;
                if spins < Self::HOT_SPINS {
                    spin_pause();
                } else {
                    yield_thread();
                }
            }

            self.awake.fetch_sub(1, Ordering::SeqCst);
        }
    }
}

pub struct WorkerPool {
    /// Threads including the caller of `run`.
    thread_count: usize,
    shared: Arc<WorkerPoolShared>,
    threads: Vec<B3Thread>,
    epoch: u32,
    is_running: bool,
}

impl WorkerPool {
    /// Largest number of blocks a stage may have.
    pub fn max_blocks() -> usize {
        WorkerPoolShared::MAX_BLOCKS
    }

    pub fn new(thread_count: usize) -> Self {
        let thread_count = thread_count.max(1);
        let shared = Arc::new(WorkerPoolShared::new(thread_count));
        let mut threads = Vec::with_capacity(thread_count.saturating_sub(1));
        for index in 1..thread_count {
            let shared = Arc::clone(&shared);
            threads.push(B3Thread::new(
                Some(&format!("natsel worker {index}")),
                move || shared.worker_main(index),
            ));
        }
        Self {
            thread_count,
            shared,
            threads,
            epoch: 0,
            is_running: false,
        }
    }

    pub fn thread_count(&self) -> usize {
        self.thread_count
    }

    pub fn is_running(&self) -> bool {
        self.is_running
    }

    /// Wake the workers and wait until all of them are spinning. They spin until `end`.
    pub fn begin<F>(&mut self, handler: F)
    where
        F: Fn(WorkerStage, usize, usize) + 'static,
    {
        if self.thread_count == 1 {
            // The handler is invoked directly by `run`; keep it type-erased for symmetry.
            let handler: Handler = {
                let wrapped = SendFn(handler);
                Arc::new(move |stage, block, thread| (wrapped.0)(stage, block, thread))
            };
            *self.shared.handler.lock().unwrap_or_else(|e| e.into_inner()) = Some(handler);
            self.is_running = true;
            return;
        }

        let wrapped = SendFn(handler);
        let handler: Handler =
            Arc::new(move |stage, block, thread| (wrapped.0)(stage, block, thread));
        *self.shared.handler.lock().unwrap_or_else(|e| e.into_inner()) = Some(handler);
        self.shared.running.store(true, Ordering::SeqCst);
        for _ in 0..self.threads.len() {
            self.shared.wake.signal();
        }
        self.is_running = true;
        while self.shared.awake.load(Ordering::Acquire) < self.threads.len() as i32 {
            spin_pause();
        }
    }

    /// Let the workers sleep again. Returns once none of them is spinning.
    pub fn end(&mut self) {
        self.shared.running.store(false, Ordering::SeqCst);
        let mut spins = 0;
        while self.shared.awake.load(Ordering::Acquire) != 0 {
            spins += 1;
            if spins > 64 {
                yield_thread();
            }
        }
        *self.shared.handler.lock().unwrap_or_else(|e| e.into_inner()) = None;
        self.is_running = false;
    }

    /// Run all blocks of a stage on the pool and the calling thread. Returns when all are done.
    pub fn run(&mut self, stage: WorkerStage) {
        if stage.block_count <= 0 {
            return;
        }
        assert!(stage.block_count as usize <= WorkerPoolShared::MAX_BLOCKS);
        self.epoch = self.epoch.wrapping_add(1);
        *self.shared.stage.lock().unwrap_or_else(|e| e.into_inner()) = stage;
        self.shared.completed.store(0, Ordering::Relaxed);
        self.shared.next_block.store(0, Ordering::Relaxed);
        self.shared.current.store(self.epoch, Ordering::Release);

        let handler = self
            .shared
            .handler
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
            .expect("WorkerPool::run called without begin");

        self.shared.work(&handler, stage, 0);

        if self.thread_count > 1 {
            let mut spins = 0;
            while self.shared.completed.load(Ordering::Acquire) < stage.block_count as usize {
                spins += 1;
                if spins < WorkerPoolShared::HOT_SPINS {
                    spin_pause();
                } else {
                    yield_thread();
                }
            }
        }
    }
}

impl Drop for WorkerPool {
    fn drop(&mut self) {
        if self.is_running {
            self.end();
        }
        self.shared.shutdown.store(true, Ordering::SeqCst);
        for _ in 0..self.threads.len() {
            self.shared.wake.signal();
        }
        for thread in &mut self.threads {
            thread.join();
        }
    }
}
