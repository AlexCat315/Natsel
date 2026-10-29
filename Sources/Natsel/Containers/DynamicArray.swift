/// A growable array with explicit storage ownership. Elements live in raw storage owned by the
/// array and are moved bitwise; the array must be destroyed explicitly. Pointers returned by
/// `pointer(_:)` stay valid until the array grows.
struct DynamicArray<Element: BitwiseCopyable>: BitwiseCopyable {
    var data: UnsafeMutablePointer<Element>?
    var count: Int32 = 0
    var capacity: Int32 = 0

    init() {}

    /// Create with an initial capacity.
    init(capacity n: Int32) {
        data = n > 0 ? b3GrowTyped(nil as UnsafeMutablePointer<Element>?, 0, n) : nil
        count = 0
        capacity = n
    }

    mutating func destroy() {
        b3FreeTyped(data, capacity)
        data = nil
        count = 0
        capacity = 0
    }

    mutating func reserve(_ n: Int32) {
        if capacity < n {
            data = b3GrowTyped(data, capacity, n)
            capacity = n
        }
    }

    mutating func resize(_ n: Int32) {
        reserve(n)
        count = n
    }

    /// Push a new element by value.
    @inline(__always)
    mutating func push(_ value: Element) {
        if count >= capacity {
            let newCapacity: Int32 = capacity == 0 ? 8 : 2 * capacity
            data = b3GrowTyped(data, capacity, newCapacity)
            capacity = newCapacity
        }
        data![Int(count)] = value
        count += 1
    }

    /// Get a pointer to an element.
    @inline(__always)
    func pointer(_ index: Int32) -> UnsafeMutablePointer<Element> {
        b3Assert(0 <= index && index < count)
        return data! + Int(index)
    }

    @inline(__always)
    subscript(index: Int32) -> Element {
        unsafeAddress {
            b3Assert(0 <= index && index < count)
            return UnsafePointer(data! + Int(index))
        }
        nonmutating unsafeMutableAddress {
            b3Assert(0 <= index && index < count)
            return data! + Int(index)
        }
    }

    /// Create a new uninitialized element and return a pointer to it.
    @inline(__always)
    mutating func emplace() -> UnsafeMutablePointer<Element> {
        growForEmplace()
        let p = data! + Int(count)
        count += 1
        return p
    }

    /// Add an uninitialized element and return its index.
    @inline(__always)
    mutating func addIndex() -> Int32 {
        growForEmplace()
        count += 1
        return count - 1
    }

    @inline(__always)
    private mutating func growForEmplace() {
        if count >= capacity {
            let newCapacity: Int32 = capacity == 0 ? 16 : 2 * capacity
            data = b3GrowTyped(data, capacity, newCapacity)
            capacity = newCapacity
        }
    }

    /// Remove the last element and return it by value.
    @inline(__always)
    mutating func pop() -> Element {
        b3Assert(0 < count)
        count -= 1
        return data![Int(count)]
    }

    /// Append a contiguous run of values.
    mutating func append(_ src: UnsafePointer<Element>, _ n: Int32) {
        if count + n > capacity {
            let req = count + n
            let newCapacity: Int32 = req > 2 ? req + (req >> 1) : 8
            data = b3GrowTyped(data, capacity, newCapacity)
            capacity = newCapacity
        }
        if n > 0 {
            (data! + Int(count)).update(from: src, count: Int(n))
        }
        count += n
    }

    /// Zero the entire allocated buffer (capacity, not just count).
    func memZero() {
        if capacity > 0 {
            UnsafeMutableRawPointer(data!).initializeMemory(
                as: UInt8.self, repeating: 0, count: Int(capacity) * MemoryLayout<Element>.stride
            )
        }
    }

    /// Remove an element by swapping with the last element. If the index is the last element
    /// it returns nullIndex, otherwise it returns the index of the last element (which is now
    /// out of bounds).
    @inline(__always)
    mutating func removeSwap(_ index: Int32) -> Int32 {
        b3Assert(0 <= index && index < count, "Array index out of bounds")

        count -= 1
        if index != count {
            data![Int(index)] = data![Int(count)]
            return count
        }

        return B3.nullIndex
    }

    @inline(__always)
    mutating func clear() {
        count = 0
    }

    var byteCount: Int32 {
        capacity * Int32(MemoryLayout<Element>.stride)
    }
}
