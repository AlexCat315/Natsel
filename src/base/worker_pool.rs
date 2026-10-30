//! A pool of persistent worker threads for fine grained parallel stages.
//!
//! Between [`WorkerPool::begin`] and [`WorkerPool::end`] the workers spin on a stage counter so a
//! new stage starts within microseconds; outside they sleep on a semaphore. A stage is a number of
//! blocks. The main thread publishes a stage, works on it too, and waits until every block is done.
//!
//! Each block has a claim word on its own cache line holding the epoch of the stage that last ran
//! it. A thread claims a block by raising that word to the current epoch, which only succeeds while
//! the stored epoch is older, so a worker that is late for one stage can never run a block of a
//! newer one. Threads start at their own share of the blocks and then steal around the ring, which
//! keeps them off each other's cache lines.

use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, Ordering};
use std::sync::Arc;

use crate::base::platform::{spin_pause, yield_thread, B3Semaphore, B3Thread};

/// The plain data describing one stage. Copied by value; a torn read by a late worker is harmless
/// because its claims fail.
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

impl<F: Fn(WorkerStage, usize, usize)> SendFn<F> {
    /// Invoke through `&self` so a closure that captures a `SendFn` captures the wrapper rather
    /// than the bare `F` field: Rust's precise capture would otherwise require `F: Send + Sync`,
    /// defeating the erasure `SendFn` exists for.
    #[inline]
    fn call(&self, stage: WorkerStage, block: usize, thread: usize) {
        (self.0)(stage, block, thread);
    }
}

/// Stage descriptions by epoch, so publishing a stage never overwrites the one a late worker may
/// still be reading. The index is always `epoch & 3`.
struct StageSlots(UnsafeCell<[WorkerStage; 4]>);

// SAFETY: slot `epoch & 3` is written by the caller of `run` before it publishes `epoch`, and is
// only read by workers after they load an epoch that selects it. Distinct epochs four apart cannot
// overlap because a stage completes before the next one is published.
unsafe impl Sync for StageSlots {}

/// The handler, written by `begin` before `running` is set and cleared by `end` after the workers
/// stopped.
struct HandlerSlot(UnsafeCell<Option<Handler>>);

// SAFETY: every access is ordered by `running` (see `WorkerPoolShared::worker_main`/`begin`/`end`).
unsafe impl Sync for HandlerSlot {}

struct WorkerPoolShared {
    wake: B3Semaphore,
    /// Epoch of the published stage.
    current: AtomicU32,
    completed: AtomicI32,
    thread_count: usize,

    /// One claim word per block, each on its own cache line.
    claims: Box<[AtomicU32]>,

    running: AtomicBool,
    shutdown: AtomicBool,
    /// Workers that consumed a wake signal and have not gone back to sleep.
    awake: AtomicI32,

    stages: StageSlots,
    handler: HandlerSlot,
}

impl WorkerPoolShared {
    const MAX_BLOCKS: usize = 256;
    const CLAIM_STRIDE: usize = 16;

    /// Busy wait iterations before a worker starts yielding.
    const HOT_SPINS: usize = 1 << 16;

    fn new(thread_count: usize) -> Self {
        Self {
            wake: B3Semaphore::new(0),
            current: AtomicU32::new(0),
            completed: AtomicI32::new(0),
            thread_count,
            claims: (0..Self::MAX_BLOCKS * Self::CLAIM_STRIDE)
                .map(|_| AtomicU32::new(0))
                .collect(),
            running: AtomicBool::new(false),
            shutdown: AtomicBool::new(false),
            awake: AtomicI32::new(0),
            stages: StageSlots(UnsafeCell::new([WorkerStage::default(); 4])),
            handler: HandlerSlot(UnsafeCell::new(None)),
        }
    }

    #[inline]
    fn claim_word(&self, block: usize) -> &AtomicU32 {
        &self.claims[block * Self::CLAIM_STRIDE]
    }

    /// Claim and run blocks of stage `epoch` until none are left, starting at this thread's share.
    fn work(&self, epoch: u32, stage: WorkerStage, thread: usize, handler: &Handler) {
        let count = stage.block_count as usize;
        if count == 0 {
            return;
        }
        let first = thread * count / self.thread_count;
        for k in 0..count {
            let mut block = first + k;
            if block >= count {
                block -= count;
            }
            let word = self.claim_word(block);
            let seen = word.load(Ordering::Relaxed);
            // Wrapping comparison: claimable only while the stored epoch is older.
            if (epoch.wrapping_sub(seen) as i32) <= 0 {
                continue;
            }
            if word
                .compare_exchange(seen, epoch, Ordering::AcqRel, Ordering::Relaxed)
                .is_ok()
            {
                handler(stage, block, thread);
                self.completed.fetch_add(1, Ordering::Release);
            }
        }
    }

    fn worker_main(self: &Arc<Self>, thread: usize) {
        loop {
            self.wake.wait();
            if self.shutdown.load(Ordering::SeqCst) {
                return;
            }
            self.awake.fetch_add(1, Ordering::SeqCst);

            // SAFETY: `begin` stores the handler before `running` becomes true and `end` clears it
            // only after `awake` reaches zero, so the clone is ordered after the store and before
            // the clear.
            let handler = unsafe { (*self.handler.0.get()).clone() };

            if let Some(handler) = handler {
                let mut seen = self.current.load(Ordering::Acquire);
                let mut spins = 0;
                while self.running.load(Ordering::Acquire) {
                    let current = self.current.load(Ordering::Acquire);
                    if current != seen {
                        seen = current;
                        // SAFETY: the slot is only rewritten by a later epoch, four stages apart,
                        // after this stage has completed.
                        let stage = unsafe { (*self.stages.0.get())[(current & 3) as usize] };
                        self.work(current, stage, thread, &handler);
                        spins = 0;
                        continue;
                    }
                    // Stay on the core through the short serial gaps of a step; only a long gap
                    // (about a millisecond) falls back to yielding.
                    spins += 1;
                    if spins < Self::HOT_SPINS {
                        spin_pause();
                    } else {
                        yield_thread();
                    }
                }
            } else {
                while self.running.load(Ordering::Acquire) {
                    spin_pause();
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

    /// Wake the workers and wait until all of them are spinning. They spin until `end`. The handler
    /// gets the stage, the block, and the index of the thread running it (zero for the caller of
    /// `run`).
    pub fn begin<F>(&mut self, handler: F)
    where
        F: Fn(WorkerStage, usize, usize) + 'static,
    {
        assert!(!self.is_running);
        let wrapped = SendFn(handler);
        let handler: Handler =
            Arc::new(move |stage, block, thread| wrapped.call(stage, block, thread));
        // SAFETY: no worker reads the handler while `running` is false.
        unsafe { *self.shared.handler.0.get() = Some(handler) };
        self.shared.running.store(true, Ordering::SeqCst);
        for _ in 0..self.threads.len() {
            self.shared.wake.signal();
        }
        self.is_running = true;
        // Waking from the semaphore takes tens of microseconds. Wait for every worker so the first
        // stage is shared instead of mostly run by this thread.
        while self.shared.awake.load(Ordering::Acquire) < self.threads.len() as i32 {
            spin_pause();
        }
    }

    /// Let the workers sleep again. Returns once none of them is spinning.
    pub fn end(&mut self) {
        assert!(self.is_running);
        self.shared.running.store(false, Ordering::SeqCst);
        let mut spins = 0;
        while self.shared.awake.load(Ordering::Acquire) != 0 {
            spins += 1;
            if spins > 64 {
                yield_thread();
            }
        }
        // SAFETY: every worker has left the spin loop, so nobody holds a clone from this cycle.
        unsafe { *self.shared.handler.0.get() = None };
        self.is_running = false;
    }

    /// Run all blocks of a stage on the pool and the calling thread. Returns when all are done.
    pub fn run(&mut self, stage: WorkerStage) {
        if stage.block_count <= 0 {
            return;
        }
        assert!(stage.block_count as usize <= WorkerPoolShared::MAX_BLOCKS);
        self.epoch = self.epoch.wrapping_add(1);
        // SAFETY: this epoch's slot is never the one the previous three stages used, and those
        // stages have completed.
        unsafe {
            (*self.shared.stages.0.get())[(self.epoch & 3) as usize] = stage;
        }
        self.shared.completed.store(0, Ordering::Relaxed);
        self.shared.current.store(self.epoch, Ordering::Release);

        // SAFETY: `begin` installed the handler and no worker clears it.
        let handler = unsafe { (*self.shared.handler.0.get()).clone() }
            .expect("WorkerPool::run called without begin");
        self.shared.work(self.epoch, stage, 0, &handler);

        let mut spins = 0;
        while self.shared.completed.load(Ordering::Acquire) < stage.block_count {
            spins += 1;
            if spins < WorkerPoolShared::HOT_SPINS {
                spin_pause();
            } else {
                yield_thread();
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
