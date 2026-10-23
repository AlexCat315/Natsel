/// Insertion order is explicit; hash-table iteration never determines serialized order.
struct CompoundInternTable<Value> {
    private var indices: [[UInt8]: Int] = [:]
    private(set) var values: [Value] = []

    mutating func insert(_ value: Value, bytes: [UInt8]) -> Int {
        if let index = indices[bytes] { return index }
        let index = values.count
        indices[bytes] = index
        values.append(value)
        return index
    }
}

func geometryBytes(_ base: UnsafeRawPointer, _ byteCount: Int32) -> [UInt8] {
    Array(UnsafeBufferPointer(start: base.assumingMemoryBound(to: UInt8.self), count: Int(byteCount)))
}
