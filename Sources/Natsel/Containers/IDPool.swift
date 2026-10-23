struct IDPool: BitwiseCopyable {
    var freeArray = DynamicArray<Int32>()
    var nextIndex: Int32 = 0

    init() {}

    static func create() -> IDPool {
        var pool = IDPool()
        pool.freeArray.reserve(32)
        return pool
    }

    mutating func destroy() {
        freeArray.destroy()
        self = IDPool()
    }

    mutating func alloc() -> Int32 {
        let count = freeArray.count
        if count > 0 {
            return freeArray.pop()
        }

        let id = nextIndex
        nextIndex += 1
        return id
    }

    mutating func free(_ id: Int32) {
        b3Assert(nextIndex > 0)
        b3Assert(0 <= id && id < nextIndex)

        // todo does not work with assertion above
        // should probably be `id == pool->nextIndex - 1`
        if id == nextIndex {
            nextIndex -= 1
            return
        }

        freeArray.push(id)
    }

    func validateFree(_ id: Int32) {
        guard b3EnableValidation else { return }
        let freeCount = freeArray.count
        var i: Int32 = 0
        while i < freeCount {
            if freeArray[i] == id {
                return
            }
            i += 1
        }

        b3Assert(false)
    }

    var idCount: Int32 { nextIndex - freeArray.count }

    var idCapacity: Int32 { nextIndex }

    var byteCount: Int32 { freeArray.byteCount }
}
