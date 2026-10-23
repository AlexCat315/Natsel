// Internal header for the owned geometry allocation.
struct HeightFieldData: Sendable, BitwiseCopyable {
    /// Version must be first.
    var version: UInt64 = 0

    /// Hash of this height field.
    var hash: UInt64 = 0

    /// The total number of bytes for this height field.
    var byteCount: Int32 = 0

    /// The local axis-aligned bounding box.
    var aabb = AABB()

    /// The minimum y value.
    var minHeight: Float = 0

    /// The maximum y value
    var maxHeight: Float = 0

    /// The quantization scale.
    var heightScale: Float = 0

    /// The overall scale.
    var scale = Vec3(1, 1, 1)

    /// The number of grid columns along the local x-axis.
    var columnCount: Int32 = 0

    /// The number of grid rows along the local z-axis.
    var rowCount: Int32 = 0

    /// Offset of the compressed height array in bytes from the struct address.
    var heightsOffset: Int32 = 0

    /// Offset of the material index array in bytes from the struct address.
    var materialOffset: Int32 = 0

    /// Offset of the flag array in bytes from the struct address.
    var flagsOffset: Int32 = 0

    /// Triangle winding.
    var clockwise: UInt8 = 0

    /// Explicit padding.
    var padding = InlineArray<7, UInt8>(repeating: 0)

    init() {}
}

