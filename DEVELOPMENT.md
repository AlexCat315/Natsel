# Natsel development

Natsel is a Swift physics engine. Changes are evaluated against its own behavior, correctness
requirements, and performance history.

## Platforms and structure

The package uses Swift 6.4 and targets macOS 26 and iOS 26. The platform layer also provides
Unix/Android and Windows threading primitives and portable SIMD transposes.

- `Base/`, `Math/`, and `Containers/`: allocation, threading, hashing, arithmetic, and storage.
- `Types/` and `Collision/`: public definitions, geometry, collision detection, and queries.
- `Simulation/`: bodies, contacts, constraint graphs, joints, CCD, events, and world state.
- `Character/`, `Vehicle/`, `Skeleton/`, and `SoftBody/`: specialized simulation systems.

Public APIs use Swift types such as `Vec3`, `Transform`, `BodyID`, and `World`.
Geometry objects own their packed storage; identical geometry can be shared through the world.
Serialized allocations are zeroed before hashing so padding does not affect content identity.

## Determinism and correctness

The same version, configuration, and inputs must produce repeatable results. Worker count must
not change the simulation. Scalar and SIMD paths preserve the evaluation order within each lane,
including operand order, expression association, signed zero, and tie-breaking rules.

Tests compare independently created Natsel scenes, scalar/SIMD execution, worker counts, and
save/restore sequences. Analytical geometry and physics checks cover collision, constraints,
momentum, energy, and replay quality. Opaque state hashes from another implementation are not
part of the test contract. Intentional changes to evaluation or solver order may change hashes
between versions and must be evaluated alongside correctness tests.

```sh
swift test
swift test -c release
```

## Performance

Run only the scenes relevant to the change. Compare two Natsel versions using the same machine
and settings, outside unit tests. See [PERFORMANCE.md](PERFORMANCE.md) for the procedure and
historical results.

## Website

The Vue website lives in `Website/`. Run `npm run build` there to check types and production assets.
