import Synchronization
#if canImport(Darwin)
internal import Darwin
#endif

// A pool of persistent worker threads for fine grained parallel stages (b3SolverStage in C).
//
// Between `begin` and `end` the workers spin on a stage counter so a new stage starts within
// microseconds; outside they sleep on a semaphore. A stage is a number of blocks. The main thread
// publishes a stage, works on it too, and waits until every block is done.
//
// Each block has a claim word on its own cache line holding the epoch of the stage that last ran
// it. A thread claims a block by raising that word to the current epoch, which only succeeds while
// the stored epoch is older, so a worker that is late for one stage can never run a block of a newer
// one. Threads start at their own share of the blocks and then steal around the ring, which keeps
// them off each other's cache lines.

/// The plain data describing one stage. Copied by value; a torn read by a late worker is harmless
/// because its claims fail.
struct WorkerStage: BitwiseCopyable {
    var kind: Int32 = 0
    var color: Int32 = 0
    var blockCount: Int32 = 0
    var blockSize: Int32 = 0
    var itemCount: Int32 = 0
    var flags: Int32 = 0

    init() {}
}

/// State shared by the pool and its threads. The threads own it, so it outlives the pool object.
private final class WorkerPoolShared: @unchecked Sendable {
    let wake = B3Semaphore(0)

    /// Epoch of the published stage.
    let current = Atomic<UInt32>(0)
    let completed = Atomic<Int32>(0)
    let threadCount: Int

    /// One claim word per block, each on its own cache line.
    static let maxBlocks = 256
    static let claimStride = 16
    let claims: UnsafeMutablePointer<UInt32>

    /// Busy wait iterations before a worker starts yielding.
    static let hotSpins = 1 << 16
    let running = Atomic<Bool>(false)
    let shutdown = Atomic<Bool>(false)
    /// Workers that consumed a wake signal and have not gone back to sleep.
    let awake = Atomic<Int32>(0)

    /// Stage descriptions by epoch, so publishing a stage never overwrites the one a late worker
    /// may still be reading.
    let stages = UnsafeMutablePointer<WorkerStage>.allocate(capacity: 4)

    /// Written by `begin` before `running` is set and cleared by `end` after the workers stopped.
    nonisolated(unsafe) var handler: ((WorkerStage, Int, Int) -> Void)?

    init(threadCount: Int) {
        self.threadCount = threadCount
        stages.initialize(repeating: WorkerStage(), count: 4)
        let words = Self.maxBlocks * Self.claimStride
        claims = .allocate(capacity: words)
        claims.initialize(repeating: 0, count: words)
    }

    deinit {
        stages.deinitialize(count: 4)
        stages.deallocate()
        claims.deallocate()
    }

    @inline(__always)
    private func claimWord(_ block: Int) -> UnsafeMutablePointer<Atomic<UInt32>> {
        UnsafeMutableRawPointer(claims + block * Self.claimStride).assumingMemoryBound(to: Atomic<UInt32>.self)
    }

    /// Claim and run blocks of stage `epoch` until none are left, starting at this thread's share.
    func work(_ epoch: UInt32, _ stage: WorkerStage, _ thread: Int) {
        guard let handler else { return }
        let count = Int(stage.blockCount)
        let first = thread * count / threadCount
        for k in 0..<count {
            var block = first + k
            if block >= count { block -= count }
            let word = claimWord(block)
            let seen = word.pointee.load(ordering: .relaxed)
            // Wrapping comparison: claimable only while the stored epoch is older.
            guard Int32(bitPattern: epoch &- seen) > 0 else { continue }
            let (exchanged, _) = word.pointee.compareExchange(
                expected: seen, desired: epoch, ordering: .acquiringAndReleasing)
            if exchanged {
                handler(stage, block, thread)
                completed.add(1, ordering: .releasing)
            }
        }
    }

    func workerMain(_ thread: Int) {
        #if canImport(Darwin)
        // Solver stages are short and synchronized; prefer performance cores.
        pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0)
        #endif
        while true {
            wake.wait()
            if shutdown.load(ordering: .sequentiallyConsistent) { return }
            awake.add(1, ordering: .sequentiallyConsistent)

            var seen = current.load(ordering: .acquiring)
            var spins = 0
            while running.load(ordering: .acquiring) {
                let current = current.load(ordering: .acquiring)
                if current != seen {
                    seen = current
                    work(current, stages[Int(current & 3)], thread)
                    spins = 0
                    continue
                }
                // Stay on the core through the short serial gaps of a step; only a long gap
                // (about a millisecond) falls back to yielding.
                spins += 1
                if spins < Self.hotSpins { spinPause() } else { yieldThread() }
            }

            awake.add(-1, ordering: .sequentiallyConsistent)
        }
    }
}

final class WorkerPool {
    /// Threads including the caller of `run`.
    let threadCount: Int

    private let shared: WorkerPoolShared
    private var threads: [B3Thread] = []
    private var epoch: UInt32 = 0
    private(set) var isRunning = false

    /// Largest number of blocks a stage may have.
    static var maxBlocks: Int { WorkerPoolShared.maxBlocks }

    init(threadCount: Int) {
        self.threadCount = max(1, threadCount)
        let shared = WorkerPoolShared(threadCount: self.threadCount)
        self.shared = shared
        for index in 1..<self.threadCount {
            threads.append(B3Thread(name: "natsel worker \(index)") { shared.workerMain(index) })
        }
    }

    deinit {
        if isRunning { end() }
        shared.shutdown.store(true, ordering: .sequentiallyConsistent)
        for _ in threads { shared.wake.signal() }
        for thread in threads { thread.join() }
    }

    /// Wake the workers and wait until all of them are spinning. They spin until `end`. The handler
    /// gets the stage, the block, and the index of the thread running it (zero for the caller of `run`).
    func begin(_ handler: @escaping (WorkerStage, Int, Int) -> Void) {
        b3Assert(isRunning == false)
        shared.handler = handler
        shared.running.store(true, ordering: .sequentiallyConsistent)
        for _ in threads { shared.wake.signal() }
        isRunning = true
        // Waking from the semaphore takes tens of microseconds. Wait for every worker so the first
        // stage is shared instead of mostly run by this thread.
        while shared.awake.load(ordering: .acquiring) < Int32(threads.count) { spinPause() }
    }

    /// Let the workers sleep again. Returns once none of them is spinning.
    func end() {
        b3Assert(isRunning)
        shared.running.store(false, ordering: .sequentiallyConsistent)
        var spins = 0
        while shared.awake.load(ordering: .acquiring) != 0 {
            spins += 1
            if spins > 64 { yieldThread() }
        }
        shared.handler = nil
        isRunning = false
    }

    /// Run all blocks of a stage on the pool and the calling thread. Returns when all are done.
    func run(_ stage: WorkerStage) {
        guard stage.blockCount > 0 else { return }
        b3Assert(Int(stage.blockCount) <= Self.maxBlocks)
        epoch &+= 1
        shared.stages[Int(epoch & 3)] = stage
        shared.completed.store(0, ordering: .relaxed)
        shared.current.store(epoch, ordering: .releasing)

        shared.work(epoch, stage, 0)

        var spins = 0
        while shared.completed.load(ordering: .acquiring) < stage.blockCount {
            spins += 1
            if spins < WorkerPoolShared.hotSpins { spinPause() } else { yieldThread() }
        }
    }
}
