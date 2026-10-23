@inline(__always)
func keyHash(_ key: UInt64) -> UInt32 {
    var h = key
    h ^= h >> 33
    h = h &* 0xff51_afd7_ed55_8ccd
    h ^= h >> 33
    h = h &* 0xc4ce_b9fe_1a85_ec53
    h ^= h >> 33

    return UInt32(truncatingIfNeeded: h)
}

struct SetItem: BitwiseCopyable {
    var key: UInt64 = 0
    var hash: UInt32 = 0
}

private let shapeMask = UInt64(B3.maxShapes - 1)
private let childMask = UInt64(B3.maxChildShapes - 1)

@inline(__always)
func shapePairKey(_ s1: Int32, _ s2: Int32, _ c: Int32) -> UInt64 {
    let child = UInt64(truncatingIfNeeded: Int64(c)) & childMask
    if s1 < s2 {
        return ((shapeMask & UInt64(truncatingIfNeeded: Int64(s1))) << UInt64(64 - B3.shapePower))
            | ((shapeMask & UInt64(truncatingIfNeeded: Int64(s2))) << UInt64(64 - 2 * B3.shapePower)) | child
    }

    return ((shapeMask & UInt64(truncatingIfNeeded: Int64(s2))) << UInt64(64 - B3.shapePower))
        | ((shapeMask & UInt64(truncatingIfNeeded: Int64(s1))) << UInt64(64 - 2 * B3.shapePower)) | child
}

/// Open addressing hash set of non-zero 64-bit keys.
struct HashSet: BitwiseCopyable {
    var items: UnsafeMutablePointer<SetItem>?
    var capacity: UInt32 = 0
    var count: UInt32 = 0

    init() {}

    init(capacity: Int32) {
        // Capacity must be a power of 2
        if capacity > 16 {
            self.capacity = UInt32(roundUpPowerOf2(capacity))
        } else {
            self.capacity = 16
        }

        count = 0
        items = b3AllocTyped(SetItem.self, Int32(self.capacity))
        items?.initialize(repeating: SetItem(), count: Int(self.capacity))
    }

    mutating func destroy() {
        b3FreeTyped(items, Int32(capacity))
        items = nil
        count = 0
        capacity = 0
    }

    mutating func clear() {
        count = 0
        items?.update(repeating: SetItem(), count: Int(capacity))
    }

    @inline(__always)
    private func findSlot(_ key: UInt64, _ hash: UInt32) -> Int32 {
        let capacity = self.capacity
        var index = Int32(bitPattern: hash & (capacity - 1))
        let items = self.items!
        while items[Int(index)].hash != 0 && items[Int(index)].key != key {
            index = (index + 1) & Int32(bitPattern: capacity - 1)
        }

        return index
    }

    private mutating func addKeyHaveCapacity(_ key: UInt64, _ hash: UInt32) {
        let index = findSlot(key, hash)
        let items = self.items!
        b3Assert(items[Int(index)].hash == 0)

        items[Int(index)].key = key
        items[Int(index)].hash = hash
        count += 1
    }

    private mutating func grow() {
        let oldCount = count

        let oldCapacity = capacity
        let oldItems = items!

        count = 0
        // Capacity must be a power of 2
        capacity = 2 * oldCapacity
        items = b3AllocTyped(SetItem.self, Int32(capacity))
        items!.initialize(repeating: SetItem(), count: Int(capacity))

        // Transfer items into new array
        var i: UInt32 = 0
        while i < oldCapacity {
            let item = oldItems[Int(i)]
            if item.hash != 0 {
                addKeyHaveCapacity(item.key, item.hash)
            }
            i += 1
        }

        b3Assert(count == oldCount)

        b3FreeTyped(oldItems, Int32(oldCapacity))
    }

    func containsKey(_ key: UInt64) -> Bool {
        // key of zero is a sentinel
        b3Assert(key != 0)
        let hash = keyHash(key)
        let index = findSlot(key, hash)
        return items![Int(index)].key == key
    }

    /// The caller already has the hash, which lets a batch prefetch the slots before probing them.
    func containsHashedKey(_ key: UInt64, _ hash: UInt32) -> Bool {
        b3Assert(key != 0)
        b3Assert(hash == keyHash(key))
        let index = findSlot(key, hash)
        return items![Int(index)].key == key
    }

    var byteCount: Int32 {
        Int32(capacity) * Int32(MemoryLayout<SetItem>.stride)
    }

    /// Returns true if key was already in set.
    mutating func addKey(_ key: UInt64) -> Bool {
        // key of zero is a sentinel
        b3Assert(key != 0)

        let hash = keyHash(key)
        b3Assert(hash != 0)

        let index = findSlot(key, hash)
        if items![Int(index)].hash != 0 {
            // Already in set
            b3Assert(items![Int(index)].hash == hash && items![Int(index)].key == key)
            return true
        }

        if 2 * count >= capacity {
            grow()
        }

        addKeyHaveCapacity(key, hash)
        return false
    }

    /// Returns true if the key was found.
    /// See https://en.wikipedia.org/wiki/Open_addressing
    mutating func removeKey(_ key: UInt64) -> Bool {
        let hash = keyHash(key)
        var i = findSlot(key, hash)
        let items = self.items!
        if items[Int(i)].hash == 0 {
            // Not in set
            return false
        }

        // Mark item i as unoccupied
        items[Int(i)].key = 0
        items[Int(i)].hash = 0

        b3Assert(count > 0)
        count -= 1

        // Attempt to fill item i
        var j = i
        let capacity = self.capacity
        while true {
            j = (j + 1) & Int32(bitPattern: capacity - 1)
            if items[Int(j)].hash == 0 {
                break
            }

            // k is the first item for the hash of j
            let k = Int32(bitPattern: items[Int(j)].hash & (capacity - 1))

            // determine if k lies cyclically in (i,j]
            // i <= j: | i..k..j |
            // i > j: |.k..j  i....| or |....j     i..k.|
            if i <= j {
                if i < k && k <= j {
                    continue
                }
            } else {
                if i < k || k <= j {
                    continue
                }
            }

            // Move j into i
            items[Int(i)] = items[Int(j)]

            // Mark item j as unoccupied
            items[Int(j)].key = 0
            items[Int(j)].hash = 0

            i = j
        }

        return true
    }
}

extension BitSet {
    func countSetBits() -> Int32 {
        var popCount: Int32 = 0
        let blockCount = self.blockCount
        var i: UInt32 = 0
        while i < blockCount {
            popCount += popCount64(bits![Int(i)])
            i += 1
        }

        return popCount
    }
}
