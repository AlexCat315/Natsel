

// Contact storage view.
//
// The collide pass runs in parallel, so contact writes must not go through the World's
// `contacts` array: that would copy-on-write the whole buffer per write and race across workers.
// Every narrow-phase routine therefore takes a `ContactStore`, which is either a plain reference
// to the array (serial) or an unsafe buffer over it (parallel). Writes land in place either way.

struct ContactStore {
    private let buffer: UnsafeMutableBufferPointer<Contact>
    private let array: UnsafeMutablePointer<ContiguousArray<Contact>>?

    /// Wrap the world's array for serial access.
    init(_ array: inout ContiguousArray<Contact>) {
        self.array = withUnsafeMutablePointer(to: &array) { UnsafeMutablePointer($0) }
        self.buffer = UnsafeMutableBufferPointer(start: nil, count: 0)
    }

    /// Wrap a buffer for parallel access. Only elements inside the buffer may be written.
    init(_ buffer: UnsafeMutableBufferPointer<Contact>) {
        self.buffer = buffer
        self.array = nil
    }

    @inline(__always)
    func get(_ id: Int32) -> Contact {
        if buffer.count > 0 {
            return buffer[Int(id)]
        }
        return array!.pointee[Int(id)]
    }

    @inline(__always)
    func set(_ id: Int32, _ contact: Contact) {
        if buffer.count > 0 {
            buffer[Int(id)] = contact
        } else {
            array!.pointee[Int(id)] = contact
        }
    }

    @inline(__always)
    var count: Int {
        if buffer.count > 0 { return buffer.count }
        return array!.pointee.count
    }
}
