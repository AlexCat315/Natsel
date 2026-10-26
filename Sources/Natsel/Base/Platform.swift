#if canImport(Darwin)
internal import Darwin
#elseif canImport(Glibc)
internal import Glibc
#elseif canImport(Musl)
internal import Musl
#elseif canImport(Bionic)
internal import Bionic
#elseif os(Windows)
internal import WinSDK
#endif
internal import Dispatch
private import Foundation
import Synchronization

// MARK: - Timer

/// Get the absolute number of system ticks. The value is platform specific.
public func getTicks() -> UInt64 {
    #if canImport(Darwin)
    return mach_absolute_time()
    #elseif os(Windows)
    return DispatchTime.now().uptimeNanoseconds
    #else
    var ts = timespec()
    clock_gettime(CLOCK_MONOTONIC, &ts)
    return UInt64(ts.tv_sec) * 1_000_000_000 + UInt64(ts.tv_nsec)
    #endif
}

private let millisecondsPerTick: Double = {
    #if canImport(Darwin)
    var timebase = mach_timebase_info_data_t()
    mach_timebase_info(&timebase)
    // convert to ns then to ms
    return 1e-6 * Double(timebase.numer) / Double(timebase.denom)
    #else
    return 1e-6
    #endif
}()

/// Get the milliseconds passed from an initial tick value.
public func getMilliseconds(_ ticks: UInt64) -> Float {
    let ticksNow = getTicks()
    return Float(millisecondsPerTick * Double(ticksNow &- ticks))
}

/// Get the milliseconds passed from an initial tick value and reset the tick value.
public func getMillisecondsAndReset(_ ticks: inout UInt64) -> Float {
    let ticksNow = getTicks()
    let ms = Float(millisecondsPerTick * Double(ticksNow &- ticks))
    ticks = ticksNow
    return ms
}

/// Yield to be used in a busy loop.
public func yieldThread() {
    #if os(Windows)
    _ = SwitchToThread()
    #else
    sched_yield()
    #endif
}

/// Sleep the current thread for a number of milliseconds.
public func sleepThread(_ milliseconds: Int32) {
    #if os(Windows)
    Sleep(DWORD(max(0, milliseconds)))
    #else
    var ts = timespec()
    ts.tv_sec = Int(milliseconds / 1000)
    ts.tv_nsec = Int(milliseconds % 1000) * 1_000_000
    nanosleep(&ts, nil)
    #endif
}

// MARK: - Processor topology

/// Number of performance cores. On a uniform CPU this is the number of active cores. Fine grained
/// barrier stages run at the pace of their slowest thread, so they should not exceed it.
let performanceCoreCount: Int = {
    #if canImport(Darwin)
    var count: Int32 = 0
    var size = MemoryLayout<Int32>.size
    if sysctlbyname("hw.perflevel0.physicalcpu", &count, &size, nil, 0) == 0 && count > 0 {
        return Int(count)
    }
    if sysctlbyname("hw.activecpu", &count, &size, nil, 0) == 0 && count > 0 {
        return Int(count)
    }
    return 1
    #else
    return max(1, ProcessInfo.processInfo.activeProcessorCount)
    #endif
}()

// MARK: - Threading primitives

final class B3Mutex: @unchecked Sendable {
    #if os(Windows)
    private var mutex = SRWLOCK()

    init() { InitializeSRWLock(&mutex) }
    func lock() { AcquireSRWLockExclusive(&mutex) }
    func unlock() { ReleaseSRWLockExclusive(&mutex) }
    #else
    private let mutex: UnsafeMutablePointer<pthread_mutex_t>

    init() {
        mutex = .allocate(capacity: 1)
        mutex.initialize(to: pthread_mutex_t())
        pthread_mutex_init(mutex, nil)
    }

    deinit {
        pthread_mutex_destroy(mutex)
        mutex.deallocate()
    }

    func lock() {
        pthread_mutex_lock(mutex)
    }

    func unlock() {
        pthread_mutex_unlock(mutex)
    }
    #endif
}

final class B3Semaphore: @unchecked Sendable {
    private let semaphore: DispatchSemaphore
    private let initialCount: Int32

    init(_ initCount: Int32) {
        semaphore = DispatchSemaphore(value: Int(initCount))
        initialCount = initCount
    }

    deinit {
        // libdispatch aborts if the current count is less than the initial count at release time.
        // Pad with signals so the invariant always holds; no one is waiting at this point.
        for _ in 0..<initialCount {
            semaphore.signal()
        }
    }

    func wait() {
        semaphore.wait()
    }

    func signal() {
        semaphore.signal()
    }
}

final class B3Thread: @unchecked Sendable {
    private final class Start {
        let function: () -> Void
        let name: String?

        init(function: @escaping () -> Void, name: String?) {
            self.function = function
            self.name = name
        }
    }

    #if os(Windows)
    private var thread: HANDLE?
    #elseif canImport(Darwin)
    private var thread: pthread_t?
    #else
    private var thread = pthread_t()
    #endif

    /// Name may be nil, otherwise it is copied.
    init(name: String?, _ function: @escaping () -> Void) {
        let start = Unmanaged.passRetained(Start(function: function, name: name)).toOpaque()
        #if os(Windows)
        thread = CreateThread(nil, 0, { param in
            let start = Unmanaged<Start>.fromOpaque(param!).takeRetainedValue()
            start.function()
            return 0
        }, start, 0, nil)
        if thread == nil {
            Unmanaged<Start>.fromOpaque(start).release()
            preconditionFailure("Could not create physics worker thread")
        }
        #else
        pthread_create(&thread, nil, { param in
            let parameter: UnsafeMutableRawPointer? = param
            let start = Unmanaged<Start>.fromOpaque(parameter!).takeRetainedValue()
            if let name = start.name, !name.isEmpty {
                #if canImport(Darwin)
                pthread_setname_np(String(name.prefix(15)))
                #elseif canImport(Bionic)
                pthread_setname_np(pthread_self(), String(name.prefix(15)))
                #else
                Thread.current.name = String(name.prefix(15))
                #endif
            }
            start.function()
            return nil
        }, start)
        #endif
    }

    func join() {
        #if os(Windows)
        if let thread {
            _ = WaitForSingleObject(thread, DWORD.max)
            _ = CloseHandle(thread)
            self.thread = nil
        }
        #elseif canImport(Darwin)
        if let thread {
            pthread_join(thread, nil)
        }
        #else
        pthread_join(thread, nil)
        #endif
    }
}

// MARK: - Atomics on plain memory

/// Plain storage for an atomic integer. Operations go through `UnsafeMutablePointer`
/// so the containing structs stay bitwise copyable like their C counterparts.
struct AtomicInt: BitwiseCopyable {
    var value: Int32 = 0
}

struct AtomicU32: BitwiseCopyable {
    var value: UInt32 = 0
}

@inline(__always)
private func atomicCell<T: AtomicRepresentable>(_ p: UnsafeMutableRawPointer, _: T.Type) -> UnsafeMutablePointer<Atomic<T>> {
    p.assumingMemoryBound(to: Atomic<T>.self)
}

@inline(__always)
func atomicStoreInt(_ a: UnsafeMutablePointer<AtomicInt>, _ value: Int32) {
    atomicCell(UnsafeMutableRawPointer(a), Int32.self).pointee.store(value, ordering: .sequentiallyConsistent)
}

@inline(__always)
func atomicLoadInt(_ a: UnsafeMutablePointer<AtomicInt>) -> Int32 {
    atomicCell(UnsafeMutableRawPointer(a), Int32.self).pointee.load(ordering: .sequentiallyConsistent)
}

@inline(__always)
func atomicFetchAddInt(_ a: UnsafeMutablePointer<AtomicInt>, _ increment: Int32) -> Int32 {
    atomicCell(UnsafeMutableRawPointer(a), Int32.self).pointee.add(increment, ordering: .sequentiallyConsistent).oldValue
}

@inline(__always)
func atomicCompareExchangeInt(_ a: UnsafeMutablePointer<AtomicInt>, _ expected: Int32, _ desired: Int32) -> Bool {
    atomicCell(UnsafeMutableRawPointer(a), Int32.self).pointee.compareExchange(
        expected: expected, desired: desired, ordering: .sequentiallyConsistent
    ).exchanged
}

@inline(__always)
func atomicStoreU32(_ a: UnsafeMutablePointer<AtomicU32>, _ value: UInt32) {
    atomicCell(UnsafeMutableRawPointer(a), UInt32.self).pointee.store(value, ordering: .sequentiallyConsistent)
}

@inline(__always)
func atomicLoadU32(_ a: UnsafeMutablePointer<AtomicU32>) -> UInt32 {
    atomicCell(UnsafeMutableRawPointer(a), UInt32.self).pointee.load(ordering: .sequentiallyConsistent)
}

/// Relaxed load on a plain word. Lets a racing reader peek before paying for a read modify write.
@inline(__always)
func atomicLoadU32Raw(_ a: UnsafeMutablePointer<UInt32>) -> UInt32 {
    atomicCell(UnsafeMutableRawPointer(a), UInt32.self).pointee.load(ordering: .relaxed)
}

@inline(__always)
func atomicFetchOrU32(_ a: UnsafeMutablePointer<UInt32>, _ mask: UInt32) -> UInt32 {
    atomicCell(UnsafeMutableRawPointer(a), UInt32.self).pointee.bitwiseOr(mask, ordering: .sequentiallyConsistent).oldValue
}

// MARK: - Spin wait

#if arch(arm64)
@_silgen_name("llvm.aarch64.hint")
private func llvmHint(_ hint: Int32)
#elseif arch(x86_64)
@_silgen_name("llvm.x86.sse2.pause")
private func llvmPause()
#endif

/// Processor hint for a busy wait (`yield` on arm64, `pause` on x86). Unlike `sched_yield` it never
/// gives up the core, so a spinning worker sees new work within nanoseconds.
@inline(__always)
func spinPause() {
    #if arch(arm64)
    llvmHint(1)
    #elseif arch(x86_64)
    llvmPause()
    #endif
}

// MARK: - Prefetch

/// LLVM's prefetch intrinsic. The swift-level declaration lowers to `prfm` on arm64 and
/// `prefetcht0` on x86; it never faults on invalid addresses.
@_silgen_name("llvm.prefetch.p0")
private func llvmPrefetch(_ address: UnsafeRawPointer, _ rw: Int32, _ locality: Int32, _ cacheType: Int32)

/// Read prefetch with high temporal locality (b3Prefetch / __builtin_prefetch(p, 0, 3)).
@inline(__always)
func b3Prefetch(_ address: UnsafeRawPointer) {
    llvmPrefetch(address, 0, 3, 1)
}
