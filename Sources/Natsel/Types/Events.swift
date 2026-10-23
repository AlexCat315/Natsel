

// Events are buffered in the world during the time step and are available to query after the
// time step is complete. Event buffers point into world storage and stay valid until the next
// world step or until the referenced objects are destroyed.

/// A begin-touch event is generated when a shape starts to overlap a sensor shape.
public struct SensorBeginTouchEvent: Sendable, Equatable, BitwiseCopyable {
    /// The id of the sensor shape
    public var sensorShapeId = ShapeID()

    /// The id of the shape that began touching the sensor shape
    public var visitorShapeId = ShapeID()
}

/// An end touch event is generated when a shape stops overlapping a sensor shape. You will also
/// get an end event if the sensor or visitor are destroyed, so confirm the shape id is valid.
public struct SensorEndTouchEvent: Sendable, Equatable, BitwiseCopyable {
    /// The id of the sensor shape. This shape may have been destroyed.
    public var sensorShapeId = ShapeID()

    /// The id of the shape that stopped touching the sensor shape. This shape may have been destroyed.
    public var visitorShapeId = ShapeID()
}

/// Sensor events buffered in the world.
public struct SensorEvents {
    /// Sensor begin touch events
    public var beginEvents: UnsafeBufferPointer<SensorBeginTouchEvent>

    /// Sensor end touch events
    public var endEvents: UnsafeBufferPointer<SensorEndTouchEvent>
}

/// A begin-touch event is generated when two shapes begin touching.
public struct ContactBeginTouchEvent: Sendable, Equatable, BitwiseCopyable {
    /// Id of the first shape
    public var shapeIdA = ShapeID()

    /// Id of the second shape
    public var shapeIdB = ShapeID()

    /// The transient contact id. This contact may be destroyed automatically when the world is
    /// modified or simulated.
    public var contactId = ContactID()
}

/// An end touch event is generated when two shapes stop touching.
public struct ContactEndTouchEvent: Sendable, Equatable, BitwiseCopyable {
    /// Id of the first shape. This shape may have been destroyed.
    public var shapeIdA = ShapeID()

    /// Id of the second shape. This shape may have been destroyed.
    public var shapeIdB = ShapeID()

    /// Id of the contact. This contact may have been destroyed.
    public var contactId = ContactID()
}

/// A hit touch event is generated when two shapes collide with a speed faster than the hit speed threshold.
public struct ContactHitEvent: Sendable, Equatable, BitwiseCopyable {
    /// Id of the first shape
    public var shapeIdA = ShapeID()

    /// Id of the second shape
    public var shapeIdB = ShapeID()

    /// Id of the contact. This contact may have been destroyed.
    public var contactId = ContactID()

    /// Point where the shapes hit at the beginning of the time step.
    public var point = Pos()

    /// Normal vector pointing from shape A to shape B
    public var normal = Vec3()

    /// The speed the shapes are approaching. Always positive. Typically in meters per second.
    public var approachSpeed: Float = 0

    /// User material on shape A
    public var userMaterialIdA: UInt64 = 0

    /// User material on shape B
    public var userMaterialIdB: UInt64 = 0
}

/// Contact events buffered in the world.
public struct ContactEvents {
    /// Begin touch events
    public var beginEvents: UnsafeBufferPointer<ContactBeginTouchEvent>

    /// End touch events
    public var endEvents: UnsafeBufferPointer<ContactEndTouchEvent>

    /// Hit events
    public var hitEvents: UnsafeBufferPointer<ContactHitEvent>
}

/// Body move events triggered when a body moves due to simulation. Not reported for bodies moved
/// by the user. This also has a flag to indicate that the body went to sleep.
public struct BodyMoveEvent: BitwiseCopyable {
    /// The body user data.
    public var userData: UnsafeMutableRawPointer?

    /// The body transform.
    public var transform = WorldTransform()

    /// The body id.
    public var bodyId = BodyID()

    /// Did the body fall asleep this time step?
    public var fellAsleep = false
}

/// Body events buffered in the world.
public struct BodyEvents {
    /// Move events
    public var moveEvents: UnsafeBufferPointer<BodyMoveEvent>
}

/// Joint events report joints that are awake and have a force and/or torque exceeding the threshold.
public struct JointEvent: BitwiseCopyable {
    /// The joint id
    public var jointId = JointID()

    /// The user data from the joint for convenience
    public var userData: UnsafeMutableRawPointer?
}

/// Joint events buffered in the world.
public struct JointEvents {
    /// Joint events
    public var jointEvents: UnsafeBufferPointer<JointEvent>
}

/// The contact data for two shapes. By convention the manifold normal points from shape A to shape B.
public struct ContactData {
    /// The contact id. This id may become orphaned. Check it is valid before using it.
    public var contactId = ContactID()

    /// The first shape id.
    public var shapeIdA = ShapeID()

    /// The second shape id.
    public var shapeIdB = ShapeID()

    /// The contact manifolds. This points to internal data and may become invalid. Do not store it.
    /// For mesh and height-field collision there can be multiple manifolds.
    public var manifolds = UnsafeBufferPointer<Manifold>(start: nil, count: 0)

    public init() {}
}
