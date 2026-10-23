let nullName: UInt32 = 0

/// This is designed to hash small strings while avoiding collisions for a 32-bit hash.
/// Note: changing this will break recordings.
func hash32<C: Collection<UInt8>>(_ bytes: C) -> UInt32 {
    // FNV-1a
    var h: UInt32 = 0x811c_9dc5
    for byte in bytes {
        h ^= UInt32(byte)
        h = h &* 0x0100_0193
    }

    // Murmur3 finalizer to help with short strings
    h ^= h >> 16
    h = h &* 0x85eb_ca6b
    h ^= h >> 13
    h = h &* 0xc2b2_ae35
    h ^= h >> 16
    return h
}

struct NameEntry {
    // Names are used for debugging and I'm favoring simplicity and minimal storage.
    // Collisions will be logged.
    var hash: UInt32
    var name: String
}

struct NameCache {
    var entries: [NameEntry] = []
    var map: [UInt32: Int32] = [:]

    mutating func add(_ name: String?) -> UInt32 {
        guard let name, !name.isEmpty else {
            return nullName
        }

        var id = hash32(name.utf8)
        id = id == 0 ? 1 : id

        if let index = map[id] {
            if entries[Int(index)].name != name {
                // Different name, same hash
                b3Log("Hash collision on \(name)")
            }
            return id
        }

        let index = Int32(entries.count)
        entries.append(NameEntry(hash: id, name: name))
        map[id] = index
        return id
    }

    func find(_ id: UInt32) -> String? {
        guard let index = map[id] else {
            return nil
        }
        return entries[Int(index)].name
    }

    func find(_ id: UInt32, default defaultName: String) -> String {
        find(id) ?? defaultName
    }

    /// Load a name from a recording.
    mutating func load(_ id: UInt32, _ name: String?) {
        guard let name, !name.isEmpty, id != nullName else {
            return
        }

        if map[id] != nil {
            return
        }

        let index = Int32(entries.count)
        entries.append(NameEntry(hash: id, name: name))
        map[id] = index
    }
}
