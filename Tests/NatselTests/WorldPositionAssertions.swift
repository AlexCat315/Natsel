@testable import Natsel

// Existing tolerances remain in local units; don't narrow double world coordinates for assertions.
func absFloat(_ value: Double) -> Double { abs(value) }
func distance(_ a: Pos, _ b: Vec3) -> Float { Natsel.distance(a, Pos(b)) }
func distance(_ a: Vec3, _ b: Pos) -> Float { Natsel.distance(Pos(a), b) }
