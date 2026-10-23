

// These ids serve as handles to internal objects. They should be considered opaque data and
// passed by value. All ids are null when zero initialized.

/// World id references a world instance. This should be treated as an opaque handle.
public struct WorldID: Sendable, Hashable, BitwiseCopyable {
    var index1: UInt16 = 0
    var generation: UInt16 = 0

    public init() {}

    init(index1: UInt16, generation: UInt16) {
        self.index1 = index1
        self.generation = generation
    }

    public static let null = WorldID()

    public var isNull: Bool { index1 == 0 }

    /// Store a world id into a UInt32.
    public var stored: UInt32 { (UInt32(index1) << 16) | UInt32(generation) }

    /// Load a world id from a UInt32.
    public init(stored x: UInt32) {
        index1 = UInt16(truncatingIfNeeded: x >> 16)
        generation = UInt16(truncatingIfNeeded: x)
    }
}

/// Body id references a body instance. This should be treated as an opaque handle.
public struct BodyID: Sendable, Hashable, BitwiseCopyable {
    var index1: Int32 = 0
    var world0: UInt16 = 0
    var generation: UInt16 = 0

    public init() {}

    init(index1: Int32, world0: UInt16, generation: UInt16) {
        self.index1 = index1
        self.world0 = world0
        self.generation = generation
    }

    public static let null = BodyID()

    public var isNull: Bool { index1 == 0 }

    /// Store a body id into a UInt64.
    public var stored: UInt64 {
        (UInt64(UInt32(bitPattern: index1)) << 32) | (UInt64(world0) << 16) | UInt64(generation)
    }

    /// Load a body id from a UInt64.
    public init(stored x: UInt64) {
        index1 = Int32(truncatingIfNeeded: x >> 32)
        world0 = UInt16(truncatingIfNeeded: x >> 16)
        generation = UInt16(truncatingIfNeeded: x)
    }
}

/// Shape id references a shape instance. This should be treated as an opaque handle.
public struct ShapeID: Sendable, Hashable, BitwiseCopyable {
    var index1: Int32 = 0
    var world0: UInt16 = 0
    var generation: UInt16 = 0

    public init() {}

    init(index1: Int32, world0: UInt16, generation: UInt16) {
        self.index1 = index1
        self.world0 = world0
        self.generation = generation
    }

    public static let null = ShapeID()

    public var isNull: Bool { index1 == 0 }

    /// Store a shape id into a UInt64.
    public var stored: UInt64 {
        (UInt64(UInt32(bitPattern: index1)) << 32) | (UInt64(world0) << 16) | UInt64(generation)
    }

    /// Load a shape id from a UInt64.
    public init(stored x: UInt64) {
        index1 = Int32(truncatingIfNeeded: x >> 32)
        world0 = UInt16(truncatingIfNeeded: x >> 16)
        generation = UInt16(truncatingIfNeeded: x)
    }
}

/// Joint id references a joint instance. This should be treated as an opaque handle.
public struct JointID: Sendable, Hashable, BitwiseCopyable {
    var index1: Int32 = 0
    var world0: UInt16 = 0
    var generation: UInt16 = 0

    public init() {}

    init(index1: Int32, world0: UInt16, generation: UInt16) {
        self.index1 = index1
        self.world0 = world0
        self.generation = generation
    }

    public static let null = JointID()

    public var isNull: Bool { index1 == 0 }

    /// Store a joint id into a UInt64.
    public var stored: UInt64 {
        (UInt64(UInt32(bitPattern: index1)) << 32) | (UInt64(world0) << 16) | UInt64(generation)
    }

    /// Load a joint id from a UInt64.
    public init(stored x: UInt64) {
        index1 = Int32(truncatingIfNeeded: x >> 32)
        world0 = UInt16(truncatingIfNeeded: x >> 16)
        generation = UInt16(truncatingIfNeeded: x)
    }
}

/// Contact id references a contact instance. This should be treated as an opaque handle.
public struct ContactID: Sendable, Hashable, BitwiseCopyable {
    var index1: Int32 = 0
    var world0: UInt16 = 0
    var padding: Int16 = 0
    var generation: UInt32 = 0

    public init() {}

    init(index1: Int32, world0: UInt16, generation: UInt32) {
        self.index1 = index1
        self.world0 = world0
        self.generation = generation
    }

    public static let null = ContactID()

    public var isNull: Bool { index1 == 0 }

    /// Store a contact id into three UInt32 values.
    public var stored: (UInt32, UInt32, UInt32) {
        (UInt32(bitPattern: index1), UInt32(world0), generation)
    }

    /// Load a contact id from three UInt32 values.
    public init(stored values: (UInt32, UInt32, UInt32)) {
        index1 = Int32(bitPattern: values.0)
        world0 = UInt16(truncatingIfNeeded: values.1)
        padding = 0
        generation = values.2
    }
}
