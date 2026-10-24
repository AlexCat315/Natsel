/// A growable array of plain values whose elements never move while the count is unchanged and
/// that is mutated through a reference, not through Swift's array accessors. Worker threads may
/// update distinct elements concurrently; only growth must happen on one thread with no readers.
final class StableStorage<Element: BitwiseCopyable> {
    private(set) var base: UnsafeMutablePointer<Element>
    private(set) var count = 0
    private var capacity: Int

    init(capacity: Int = 16) {
        self.capacity = max(1, capacity)
        base = .allocate(capacity: self.capacity)
    }

    deinit {
        base.deallocate()
    }

    func append(_ value: Element) {
        if count == capacity {
            let newCapacity = 2 * capacity
            let newBase = UnsafeMutablePointer<Element>.allocate(capacity: newCapacity)
            newBase.initialize(from: base, count: count)
            base.deallocate()
            base = newBase
            capacity = newCapacity
        }
        (base + count).initialize(to: value)
        count += 1
    }

    subscript(_ index: Int) -> Element {
        @inline(__always) get {
            b3Assert(index >= 0 && index < count)
            return base[index]
        }
        @inline(__always) set {
            b3Assert(index >= 0 && index < count)
            base[index] = newValue
        }
    }

    /// The elements as an array, for snapshots.
    var elements: [Element] { Array(UnsafeBufferPointer(start: base, count: count)) }

    /// Replace all elements. Only on one thread with no readers.
    func replace(with elements: [Element]) {
        if elements.count > capacity {
            base.deallocate()
            capacity = max(elements.count, 2 * capacity)
            base = .allocate(capacity: capacity)
        }
        elements.withUnsafeBufferPointer {
            if let source = $0.baseAddress { base.update(from: source, count: $0.count) }
        }
        count = elements.count
    }
}
