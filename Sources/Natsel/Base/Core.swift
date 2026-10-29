import Synchronization

/// Version numbering scheme. See https://semver.org/
public struct Version: Sendable, Equatable {
    /// Significant changes
    public var major: Int32
    /// Incremental changes
    public var minor: Int32
    /// Bug fixes
    public var revision: Int32
}

/// Get the current version of the engine.
public func getVersion() -> Version { Version(major: 0, minor: 2, revision: 0) }

/// World positions use Double; local geometry, velocities, and constraint solving use Float.
public func isDoublePrecision() -> Bool { true }

/// The maximum manifold point count the library was built with.
public func getMaxManifoldPoints() -> Int32 { B3.maxManifoldPoints }

// MARK: - Assertions

/// User assert callback. Return true to break into the debugger.
public typealias AssertHandler = @Sendable (_ condition: String, _ fileName: StaticString, _ lineNumber: UInt) -> Bool

/// User log callback. Used to log warnings.
public typealias LogHandler = @Sendable (_ message: String) -> Void

nonisolated(unsafe) private var assertHandler: AssertHandler = { condition, fileName, lineNumber in
    print("NATSEL ASSERTION: \(condition), \(fileName), line \(lineNumber)")
    // return true to break to debugger
    return true
}

nonisolated(unsafe) private var logHandler: LogHandler = { message in
    print("Natsel: \(message)")
}

/// Override the default assert callback.
public func setAssertHandler(_ handler: @escaping AssertHandler) {
    assertHandler = handler
}

/// Override the default logging callback.
public func setLogHandler(_ handler: @escaping LogHandler) {
    logHandler = handler
}

@usableFromInline
func b3InternalAssert(_ condition: String, _ fileName: StaticString, _ lineNumber: UInt) {
    if assertHandler(condition, fileName, lineNumber) {
        fatalError("assertion failed: \(condition)", file: fileName, line: lineNumber)
    }
}

/// Assert that a condition is true. Active in debug builds only.
@usableFromInline @inline(__always)
func b3Assert(
    _ condition: @autoclosure () -> Bool,
    _ message: @autoclosure () -> String = "",
    file: StaticString = #fileID,
    line: UInt = #line
) {
    #if DEBUG
    if !condition() {
        b3InternalAssert(message(), file, line)
    }
    #endif
}

/// Floating point tolerance checks. Only active when built with NATSEL_VALIDATE.
@inline(__always)
func b3Validate(
    _ condition: @autoclosure () -> Bool,
    _ message: @autoclosure () -> String = "",
    file: StaticString = #fileID,
    line: UInt = #line
) {
    #if DEBUG && NATSEL_VALIDATE
    if !condition() {
        b3InternalAssert(message(), file, line)
    }
    #endif
}

/// Whether internal validation is enabled.
let b3EnableValidation: Bool = {
    #if DEBUG && NATSEL_VALIDATE
    return true
    #else
    return false
    #endif
}()

func b3Log(_ message: String) {
    logHandler(message)
}

// MARK: - Memory

/// User allocation function. The alignment is guaranteed to be a power of 2.
public typealias AllocHandler = @Sendable (_ size: Int, _ alignment: Int) -> UnsafeMutableRawPointer?

/// User free function. The size is the size passed to the matching allocation.
public typealias FreeHandler = @Sendable (_ memory: UnsafeMutableRawPointer, _ size: Int) -> Void

nonisolated(unsafe) private var allocHandler: AllocHandler?
nonisolated(unsafe) private var freeHandler: FreeHandler?

private let allocatedByteCount = Atomic<Int64>(0)

/// This allows the user to override the allocation functions. These should be set during
/// application startup.
public func setAllocator(alloc: AllocHandler?, free: FreeHandler?) {
    allocHandler = alloc
    freeHandler = free
}

/// Total bytes allocated by the engine.
public func getByteCount() -> Int64 {
    allocatedByteCount.load(ordering: .sequentiallyConsistent)
}

func b3Alloc(_ size: Int) -> UnsafeMutableRawPointer? {
    if size == 0 {
        return nil
    }

    allocatedByteCount.add(Int64(size), ordering: .sequentiallyConsistent)

    // Allocation must be a multiple of the alignment.
    let alignedSize = ((size - 1) | (B3.alignment - 1)) + 1

    if let allocHandler {
        let ptr = allocHandler(alignedSize, B3.alignment)
        b3Assert(ptr != nil)
        b3Assert(Int(bitPattern: ptr) & (B3.alignment - 1) == 0)
        return ptr
    }

    return UnsafeMutableRawPointer.allocate(byteCount: alignedSize, alignment: B3.alignment)
}

func b3Free(_ mem: UnsafeMutableRawPointer?, _ size: Int) {
    guard let mem else {
        return
    }

    if let freeHandler {
        let alignedSize = size > 0 ? ((size - 1) | (B3.alignment - 1)) + 1 : 0
        freeHandler(mem, alignedSize)
    } else {
        mem.deallocate()
    }

    allocatedByteCount.subtract(Int64(size), ordering: .sequentiallyConsistent)
}

func b3AllocZero(_ size: Int) -> UnsafeMutableRawPointer? {
    let mem = b3Alloc(size)
    mem?.initializeMemory(as: UInt8.self, repeating: 0, count: size)
    return mem
}

func b3GrowAlloc(_ oldMem: UnsafeMutableRawPointer?, _ oldSize: Int, _ newSize: Int) -> UnsafeMutableRawPointer? {
    b3Assert(newSize > oldSize)
    let newMem = b3Alloc(newSize)
    if oldSize > 0, let oldMem, let newMem {
        newMem.copyMemory(from: oldMem, byteCount: oldSize)
        b3Free(oldMem, oldSize)
    }
    return newMem
}

func b3GrowAllocZeroed(_ oldMem: UnsafeMutableRawPointer?, _ oldSize: Int, _ newSize: Int) -> UnsafeMutableRawPointer? {
    let newMem = b3GrowAlloc(oldMem, oldSize, newSize)
    (newMem! + oldSize).initializeMemory(as: UInt8.self, repeating: 0, count: newSize - oldSize)
    return newMem
}

/// Typed allocation of `count` uninitialized elements (B3_ALLOC).
@inline(__always)
func b3AllocTyped<T: BitwiseCopyable>(_ type: T.Type, _ count: Int32) -> UnsafeMutablePointer<T>? {
    b3Alloc(Int(count) * MemoryLayout<T>.stride)?.bindMemory(to: T.self, capacity: Int(count))
}

/// Typed free of `count` elements (B3_FREE).
@inline(__always)
func b3FreeTyped<T: BitwiseCopyable>(_ mem: UnsafeMutablePointer<T>?, _ count: Int32) {
    b3Free(UnsafeMutableRawPointer(mem), Int(count) * MemoryLayout<T>.stride)
}

/// Typed growth of a buffer from `oldCount` to `newCount` elements (B3_GROW).
@inline(__always)
func b3GrowTyped<T: BitwiseCopyable>(_ mem: UnsafeMutablePointer<T>?, _ oldCount: Int32, _ newCount: Int32) -> UnsafeMutablePointer<T>? {
    let stride = MemoryLayout<T>.stride
    return b3GrowAlloc(UnsafeMutableRawPointer(mem), Int(oldCount) * stride, Int(newCount) * stride)?
        .bindMemory(to: T.self, capacity: Int(newCount))
}

/// Typed zeroed growth of a buffer from `oldCount` to `newCount` elements (B3_GROW_ZERO).
@inline(__always)
func b3GrowZeroTyped<T: BitwiseCopyable>(_ mem: UnsafeMutablePointer<T>?, _ oldCount: Int32, _ newCount: Int32) -> UnsafeMutablePointer<T>? {
    let stride = MemoryLayout<T>.stride
    return b3GrowAllocZeroed(UnsafeMutableRawPointer(mem), Int(oldCount) * stride, Int(newCount) * stride)?
        .bindMemory(to: T.self, capacity: Int(newCount))
}
