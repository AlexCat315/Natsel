# Natsel development and porting notes

## Current development direction

Natsel is developed as an independent physics engine. Compatibility with the old C reference
and prior-version state hashes is a diagnostic, not a requirement for every optimization.
Intentional changes to floating-point evaluation or solver order may change those hashes.
The hard requirements are repeatability within the same version/configuration, worker-count
determinism, and independently tested collision, constraint, conservation, and replay quality.
Document intentional compatibility changes together with their performance and quality results.

The porting rules below describe the original C-compatible implementation and remain useful
when working on paths whose exact compatibility is still being measured.

## Platform and style

- Minimum deployment target: macOS 26 / iOS 26 (needed for `InlineArray`; `Synchronization`
  atoms need macOS 15).
- Public API is Swift-idiomatic: `Vec3`, `Transform`, `BodyID`, `World`, and free functions such
  as `makeBoxHull` / `computeCosSin` instead of the C `b3*` prefixes.
- Internal C-isms keep their original names (`b3Assert`, `b3Validate`, `b3Alloc`, `hash64NonZero`)
  so the ported code reads against the C sources.

The platform layer uses POSIX primitives on Unix/Android and native Windows locks and threads.
Platforms without Apple's `simd` module use a lane-preserving portable wide transpose.
Dispatch callbacks explicitly describe their Sendable captures and disjoint buffer access.

`World.continuousImpacts()` exposes transient CCD impacts requested by contact/hit event flags,
including impacts resolved before a persistent manifold exists. It is independent of the hit-speed
threshold, resets each step, and participates in save/restore. Ordinary begin/end contact semantics
are unchanged. Static CCD also supplies thresholded hit events, matching moving-pair CCD.

## Original C-parity rules

- Preserve operand order. `dot(a, b)` is always `a.x*b.x + a.y*b.y + a.z*b.z` (left associative),
  matching both `b3Dot` and `b3DotW4`.
- Preserve expression association. For example the hull bulk properties are accumulated exactly as
  in `b3UpdateHullBulkProperties`.
- Do not introduce FMA contraction: the C SIMD helpers explicitly avoid real FMA.
- Wide (SIMD) C code that selects between near-ties must be ported as the same lane-wise
  algorithm, not as a naive scalar maximum. `findHullSupportVertex` is an example where the
  result is provably the same as a forward scan; `b3GetSupportWide` embeds the vertex index in
  the low mantissa bits and must be ported literally.
- Serialized blobs (hull, mesh, height field, compound) are zeroed before the content hash is
  computed so struct padding cannot leak into the hash.

## Layering

1. `Base/` — constants, core allocation/assert/log, rapidhash, platform threading.
2. `Math/` — scalar helpers, vec/quat/matrix, AABB helpers, validity checks.
3. `Containers/` — arena, bitset, dynamic array, hash set, ID pool, name cache, quick sort.
4. `Types/` — public POD types mirroring `include/box3d/types.h`.
5. `Collision/` — internal math, AABB ray cast, hull storage and builder, manifold clipping,
   distance (GJK), convex and triangle manifolds, compound/mesh/height field collision,
   shape AABBs, broad phase and dynamic tree.
6. `Simulation/` — body, contact, island, constraint graph, joints, solver sets, solvers.
7. `World/` — world creation/step/queries, sensors, movers, timer, scheduler, recording,
   replay, snapshots.

Each layer must build and pass `swift test` before moving to the next.

## Ownership model

`Hull` is a `final class` that owns one zeroed allocation holding the `HullData` header followed
by the trailing arrays, mirroring the C layout. The header offsets are computed by
`HullLayout` from `MemoryLayout` sizes, so the layout is self-consistent even though it is not
byte-identical to the C struct. C's ref-counted hull map becomes Swift reference counting plus
an explicit content-keyed database in the world so identical hulls still intern to one instance.
Meshes, height fields, and compounds follow the same pattern.

## Verification checkpoint — 2026-10-23

The workspace already contained a partial port before this checkpoint. Baseline: 73 tests
passed. This checkpoint adds foundation verification, not a completed engine port:

- Base: 272 rapidhash outputs generated directly from Box3D's `src/rapidhash.h` using
  Clang `-O2`. Lengths 0–256 cover every short-input and tail branch; additional lengths
  cover repeated 112-byte blocks through 4097 bytes. Unaligned reads are checked too.
  Input byte `i` is `(i * 131 + 17) & 255`; see `Tests/NatselTests/Base/HashReference.swift`.
- Determinism digest: folded little-endian words, byte tails, and wrapping seeds checked
  independently against the recurrence in `src/timer.c:b3Hash`.
- Containers: bit-set growth/union/reset, wrapped hash-set probe-chain deletion and growth,
  ID reuse order, and dynamic-array growth/swap removal covered.
- `swift test` after the base checkpoint: 75 tests passed. After containers: 79 passed.

Full engine parity is **not established** by these checks. The existing simulation reference
hashes cover small scenes; they do not replace `test/test_determinism.c`. Complete mesh,
height-field, and compound storage/collision must precede the missing world-service work
(mover, scheduler, recording/replay, snapshots). Port and run the corresponding C test
scenarios before declaring those layers complete. Release-mode floating-point parity and
iOS builds also remain unverified.

### Mesh construction and ray checkpoint — 2026-10-23

Implemented `Collision/Mesh/` in Swift:

- Spatial welding with C's signed cell hashing, neighbor traversal, and linked-list order.
- Binned SAH, median, and fallback partitions; recursive BVH nodes and triangle ordering.
- Degenerate filtering, area accumulation, material ordering, edge adjacency/concavity flags.
- Immutable owned `TriangleMesh` storage, scaled `Mesh` instances, grid generation, AABBs,
  and mesh ray casts including reflected and nonuniform scale.

`Mesh.data`, `CompoundMeshDef.meshData`, and `CompoundMesh.meshData` now retain owned
`TriangleMesh` geometry rather than requiring callers to manage raw mesh pointers.
The inverse concave-edge constants were corrected to C's 0x10/0x20/0x40.

The C oracle in `Tools/Reference/mesh_builder.c` includes the upstream `mesh.c` and links
against a separately built Box3D library. It is never linked into the Swift package.
Compile it with the command in its header, substituting the local source/build paths.
Rebuilt the C reference library before final oracle verification. Tests cover four exact
BVH/permutation digests, three complete content hashes, a grid content hash, and 240
ray outputs including normals, points, fraction bits, triangle/material indices and hit flags.
Debug suite: 84 tests passed.

Release testing also exposed existing pointer-lifetime errors in sphere/capsule overlap and
shape casts, plus a dynamic-tree rebuild exclusivity conflict. Queries now consume pointers
inside their allocation scopes, and the tree bitfield helper is static to avoid borrowing
self while mutating its storage.

Remaining mesh work: other procedural generators, overlap/shape casts, mover planes,
mesh queries, mesh contacts, and simulation/world API integration. Height fields,
compounds, world services, and full determinism scenarios remain incomplete.

Final release verification for this checkpoint: `swift test -c release` passed all 84 tests.
The tree conflict required copying the left/right node values before updating the array;
the bitfield helper alone was insufficient. The release run now covers the full existing
suite, the complete mesh hashes, and all new ray-reference cases.

### Mesh query and procedural checkpoint — 2026-10-23

Ported the remaining mesh.c collision queries: triangle/AABB SAT, proxy overlap,
shape cast, ordered triangle queries with early termination, and mover-plane collection.
All temporary proxy pointers are consumed inside their buffer scopes. Reflected meshes
preserve each C routine's particular winding choice, including SAT inputs.

Ported wave, torus, box, hollow-box, and platform generation. Grid and wave generators
share their integer topology builder; floating-point vertex generation retains C order.
World shape creation and the existing ray/overlap/shape-cast dispatch now accept owned
mesh geometry. Mesh centroid and extent calculations match shape.c, and shape destruction
releases mesh references rather than leaving them retained by recycled shape slots.

`Tools/Reference/mesh_queries.c` calls the C routines over grid, wave, and torus geometry
with three scales. Swift matches all content hashes plus 720 cases each for shape casts
(including iteration counts), transformed overlaps, ordered triangle queries, and mover
plane output (including capacity truncation). Primitive generator hashes also match C.
Debug checkpoint: `swift test` passed 87 tests in 43 suites.

An unsigned device build with `xcodebuild -scheme Natsel -destination 'generic/platform=iOS'
-derivedDataPath /tmp/natsel-ios-build CODE_SIGNING_ALLOWED=NO build` succeeded. The compiler
target was `arm64-apple-ios26.0`. This proves compilation, not device runtime determinism.

Mesh simulation contacts remain unported (`mesh_contact.c`); stepping a scene with a
mesh/convex contact is not yet supported. Height fields, compounds, world services, and
full upstream test/determinism coverage remain outstanding.

Release verification for this checkpoint: `swift test -c release` passed the same 87 tests
in 43 suites, including all nine mesh-query scenarios and procedural content hashes.

### Mesh contact and continuous-collision checkpoint — 2026-10-23

Ported the mesh path of `mesh_contact.c` into responsibility-oriented files under
`Simulation/MeshContact/`: cache refresh, triangle collision, ghost-contact filtering,
normal clustering, four-point reduction, impulse persistence, and material mixing.
Triangle caches and their query bounds now live together in `MeshContactState`.
Four-wide triangle transforms retain the per-lane C arithmetic association.

The shared contact solver now handles ordered arrays of manifolds. Warm-start impulses
are accumulated across the manifolds before modifying body velocities, matching C.
Mesh contacts enter the color solver in C's stable descending manifold-count buckets,
and their graph specs are refreshed when the narrow phase changes manifold counts.

Ported the mesh TOI query, including backside/parallel-motion rejection and the centroid
fallback. It remains in the simulation layer because it depends on runtime shapes and sweeps.

C-reference verification:

- `Tools/Reference/mesh_reduction.c`: 300 coincident, collinear, and area-selection fixtures.
- `Tools/Reference/mesh_simulation.c`: 27 scenarios (sphere/capsule/hull against grid/wave/
  reflected platform), each run for 180 steps. Nine use downward bullet velocity 150 m/s;
  nine enable sleeping. Cumulative body and contact hashes are checked every 30 steps,
  incorporating every intervening frame (4860 simulation frames total).
- Contact digests include materials, normals, anchors, feature/triangle IDs, persistence,
  friction/rolling/twist impulses, and normal impulses.
- Five lifecycle cases cover separation, shape/body deletion, sleeping deletion, and ID reuse.

These tests exposed preexisting incomplete destruction paths. Contact destruction now
unlinks graph/island entries; shape and body destruction release their attached contacts;
body destruction also releases joints/sensors and refreshes moved body-contact indices.
An orphaned sleeping solver set is no longer overwritten with its former value after removal.

Debug verification: `swift test` passed 90 tests in 46 suites. Mesh simulation no longer
hits the former unimplemented mesh-contact/CCD assertions. Height-field contact branches,
compounds, remaining world services, and full upstream test/determinism coverage remain
outstanding; the overall port is not complete.

Final checkpoint verification: `swift test -c release` passed all 90 tests in 46 suites,
and the unsigned arm64 iOS 26 device build succeeded after the contact/CCD changes.

### Height-field checkpoint — 2026-10-23

Ported `height_field.c`: owned quantized storage, decompression, adjacency flags, winding,
cell/triangle access, flat/wave factories, AABBs, DDA ray/shape casts, overlap, triangle queries,
mover planes, and the authored height-data text dump/load format. The fixed nine-decimal
text encoding follows C (and therefore has the same rounding limits for very small samples).

`ShapeGeometry.heightField` and debug height-field geometry now retain `HeightField` objects.
Raw mesh/height allocation headers are internal and live with their codecs rather than in
the public geometry API. `TriangleSurface` adapts meshes and height fields at the simulation
boundary, sharing triangle cache refresh, narrow phase, materials, and TOI without duplicating
those algorithms. Generic triangle/AABB SAT helpers now live with AABB collision code.

C-reference coverage:

- Six complete blob hashes and decoded-triangle digests: flat/wave fields, holes, clipped
  samples, and clockwise winding.
- 720 cases each of DDA rays, shape casts, transformed proxy overlaps, triangle queries,
  and mover-plane collection. C's callback-return behavior is retained for height queries.
- 54 sphere/capsule/hull simulation scenarios, each 180 steps: normal motion, downward
  150 m/s bullets, and sleeping across all six fields. Cumulative body/contact hashes cover
  every frame and are checked every 30 steps (9720 frames).
- Authored file text matches C's 655-byte fixture and digest, and loads to the matching
  geometry hash. Truncated/invalid data and missing required samples are rejected.

Debug checkpoint: `swift test` passed 95 tests in 50 suites. The unsigned iOS 26 device build
also succeeded. Compounds and remaining world services/test coverage are still outstanding;
full-engine determinism is not established by these fixtures.

Final verification after moving reusable proxy/triangle-access helpers into the collision
layer: `swift test` and `swift test -c release` both passed all 95 tests in 50 suites. The
unsigned arm64 iOS 26 device build succeeded. The shared C simulation-digest extraction
was checked by regenerating all 162 mesh checkpoint pairs; they still match the stored fixture.
Foundation imports used for file I/O are explicitly private, including the existing tree codec.

### Compound storage and traversal checkpoint — 2026-10-23

Ported compound packing, child access, immutable BVH traversal, AABBs, ray/shape casts,
overlap, ordered child queries, and mover planes. Materials and embedded hull/mesh blobs
are interned in C insertion order. Six complete compound blob hashes match C, including
mixed children and repeated geometry/materials. Shared allocation ownership lets an embedded
hull or mesh outlive its parent compound without copying or dangling pointers.

Dynamic trees and compounds now share a scoped immutable traversal view. C-reference tests
cover 720 cases each of ray casts, shape casts, overlaps, early-stop child queries, and mover
planes, including exact child/material indices. The missing hull mover routine is ported too.
The debug suite passes 99 tests in 52 suites. This is the compound collision-data layer;
compound world creation, broad-phase child contacts, solver integration, and CCD still remain.
Full upstream test migration and comprehensive engine determinism verification remain pending.

Release verification: `swift test -c release` passed all 99 tests in 52 suites. Debug tests
also pass after rejecting damaged embedded offsets. The unsigned arm64 iOS 26 build succeeds.

### Compound simulation checkpoint — 2026-10-23

Connected compounds to static world-shape creation, material ownership, shape bounds and
centroids/extents, world queries, child-pair expansion, narrow phase, and CCD. Each overlapping
child owns a persistent contact. Convex children preserve C's pair flips and child transforms;
mesh children use the existing manifold pipeline with their material-index map. Anchors are
shifted back to the compound origin before center-of-mass adjustment. Destruction removes
all child contacts and updates the compound count.

CCD follows the active compound path in `shape.c`, including its distinct mesh fallback radius
and the compound-local swept bounds. The old disabled TOI block in `compound.c` is not used.
The shared triangle CCD routine now accepts the precomputed frame, bounds, and fallback radius
so meshes/height fields and compound mesh children retain their original operation sequences.
Mesh contact hit-event flags are updated as in C, including the ordinary mesh/height path.

The C oracle `Tools/Reference/compound_simulation.c` generates 108 scenarios: six compounds,
three moving primitive types, ordinary/bullet/sleep modes, and both shape-creation orders.
Ground hit events are enabled. All cumulative body/contact hashes match every 30 steps across
180 steps each (19,440 frames). World query dispatch, material/child indices, static-only
creation, and destruction with live child contacts have additional tests. Debug verification:
102 tests in 54 suites pass.

Remaining source audit identifies world movers and non-convex sensor dispatch, custom filters,
world/body/shape/joint API completeness, scheduling/parallelism, recording/replay, snapshots,
and the full upstream test migration/determinism suite. Passing these fixtures does not prove
completion of the full engine port. The upstream `test/` directory remains the authoritative
coverage inventory, including its recording, large-world, body-query, mover, and world tests.

Final compound simulation verification: debug and release each pass all 102 tests in 54 suites;
the unsigned arm64 iOS 26 build succeeds.

### World mover and sensor checkpoint — 2026-10-23

Ported `mover.c`'s ordered plane solver and vector clipping, plus shape/world mover collision
and world mover casts. Public entry points take capsule values, origin, query filter, and
ordinary closures; temporary shape proxies stay within scoped buffer borrows. World mover
queries preserve C's per-tree order, 64-plane per-shape limit, filter behavior, origin-relative
frames, initial-overlap handling, and fraction clipping.

C-reference coverage includes 1000 plane systems (accumulated pushes, iteration count, final
delta, and clipped vector), the upstream parallel/game convergence cases and ten primitive
overlap cases, and 240 mixed-world mover queries/casts at zero and million-unit origins.
Mixed worlds cover all six geometry types, all three body trees, query masks, callback early
termination, material/child/triangle indices, and per-shape cast filter order. The generated
C fixtures are test oracles only, not production package dependencies.

Added non-convex sensor dispatch for meshes, height fields, and compounds. Ordered sensor
begin/end events match C for all six geometry types across 120 frames at both tested origins.
The comparison exposed an existing bug: end events accumulated indefinitely. End events now
have separate pending/published buffers, preserving destruction between steps and publishing
for one frame, as C does. The old unused end-event index is removed. Sensor deletion now
uses C's swap removal instead of shifting the array and leaving later indices stale.
Destruction/publication and repeated multi-sensor deletion have regression tests.

### API completeness and final verification — 2026-10-23

All remaining C public API groups are ported: body force/impulse application, disable/enable,
target transforms, contact and shape-level contact data, shape material/filter/event toggles,
names, joint inspection (reaction forces, separation, local frames, tuning), per-type joint
scalar accessors, joint events, world tuning, world bounds, explode, and the name cache.

A scripted audit of the 434 functions declared in `include/box3d/box3d.h` classifies them as:
332 mapped, 34 recording/replay tooling (deliberately omitted), 4 C callback typedefs
(replaced by closures), and 64 remaining. The 64 are Swift idiom differences rather than
gaps — `b3World_GetGravity` is `world.gravity`, `b3World_Step` is `world.step`, `b3Body_ApplyForce`
is `world.applyBodyForce`, and the like — plus debug/dump helpers and the SSE2 test switch.

Recording/replay is intentionally not ported. `world_snapshot.c` copies C structs byte for byte
behind a layout hash over 30+ `sizeof` values; reproducing that in Swift would neither match C
nor verify anything the hash tests do not already cover.

Restitution verification now mirrors `test/test_restitution.c`: head-on mass independence with
momentum conservation, a 16-sample impact phase sweep, support-column height independence,
four-point flat landings, spin about a horizontal axis, drop impulse accounting against the
momentum change, sampled normal velocity gated on hit events, elastic overshoot bounds, a
13-scene peak-energy check, multi-bounce apex monotonicity, and resting-stack stability.
`WorkerParityTest` is covered by `ParallelSolverTests`: the world state hash after every step is
identical for 1, 4, and 8 workers across seven scenes (see "Multithreaded constraint solver").

Separating-axis verification mirrors `test/test_sat.c`. A brute-force projection oracle over every
hull face and edge pair supplies the true maximum separation, and 4000 randomized samples — thin
crossed beams in deep overlap alternating with chunky boxes across the full separated-to-overlap
range — confirm the query never overstates the oracle, reproduces it inside the speculative band,
and actually reaches the edge path. The crossed-beam sweep pins the edge pair axis across seven
distances from separated to deep overlap, plus the face-axis selection cases (A's face, B's face
after A rotates, far separation, and an offset body) and a 3000-sample sweep over hulls whose
origin sits far from their own geometry.

Final verification: 361 tests in 104 suites pass in debug and release, both warning-free, and
every determinism scene hash (ragdoll 291/0xFF4C605B, wave pile 272/0x266F83B0, mesh drop
226/0xD2FF069D, query spawn 242/0xF3AD9BA6) matches the C reference.

The world-state hashes were also regenerated from the C reference binary with a fresh probe
rather than reused: an empty world comes out at the FNV seed `0xcbf29ce484222325`, and a three
box stack settles through `0x37cb0b80904ac026`, `0x949b12937583efcd`, `0x59d92287c7d3f35d`, and
`0x7e959f629051030a` at init, step 20, 40, and 60. Swift reproduces all four exactly.

The physics port is complete: 120 source files, 35,069 lines against the C engine's 51,191, with
361 tests in 104 suites covering 28 of the 29 upstream C test files. `test_recording.c` is the
one omission, by design.

## Systems beyond the Box3D port — 2026-10-24

A gap review against Jolt found Natsel complete as a Box3D port but missing several systems Jolt
users expect. These are now implemented as first class world systems. None of them change the
behavior of existing scenes: every stage returns immediately when its objects are absent, and all
determinism hashes and the 395 earlier tests still pass.

| Gap | Implementation | Files |
|---|---|---|
| Vehicles | `VehicleDef` / `createVehicle`: suspension (frequency or stiffness, preload, bump stop, force point), tire friction curves vs slip ratio and slip angle with a friction ellipse, engine torque curve, automatic/manual gearbox with clutch timing, limited slip differentials (axle and center), brakes and hand brake, anti-roll bars, pitch/roll limit; wheeled, tracked, and motorcycle (lean spring and lean steering limit) controllers; ray, sphere, and rounded cylinder wheel casts | `Vehicle/` |
| Soft bodies / cloth | Small-step XPBD: edges, shear, distance or dihedral bending, tetrahedral volume, long range attachments (euclidean or geodesic), skinning with max distance and back stop, pressure; two way coupling with rigid bodies on every geometry type; sleeping; ray casts; cloth, cube, sphere, and rope factories | `SoftBody/` |
| Character controller | `CharacterVirtual`: collide and slide, ground states, slope limit, stair walking with forward test, stick to floor, moving platforms, pushing bodies, kinematic inner body, crouch with overlap check | `Character/` |
| Buoyancy | Exact submerged volume for spheres, hulls (face clipping), closed meshes, compounds; Gauss-Legendre quadrature for capsules; Archimedes force at the center of buoyancy, quadratic drag, angular drag, currents | `Simulation/Buoyancy.swift` |
| Gear, rack and pinion, pulley | Two-body coupling joints in the constraint graph; unwrapped angle and translation tracking, optionally through the revolute/prismatic joints holding the bodies | `Simulation/CouplingJoints.swift` |
| Six DOF joint | Per axis free/limited/locked, soft limits, velocity and position motors, friction, pyramid or cone swing limits, block solves when fully locked | `Simulation/SixDOFJoint.swift` |

### Placement in the step

- Wheel casts run after the narrow phase and before the awake set is sized, because they wake the
  bodies under the wheels (waking is legal there, as in `b3UpdateContacts`). Casts only read the
  world and run in parallel across vehicles when `workerCount > 1`.
- The vehicle constraint is prepared with the contacts and solved after the joints and contacts of
  every substep: warm start (engine torque, implicit viscous clutch, anti-roll bars, accumulated
  impulses), then a biased and a relax pass. A braked tire is solved together with its brake as an
  exact 2x2 block; separate Gauss-Seidel steps converged slowly because a wheel's rotational mass
  (I/r^2, about 7 kg) is tiny next to the chassis.
- Soft bodies run after body finalization and the bullet pass. Collision planes are taken at the
  rigid bodies' end-of-step poses, and the impulses exchanged with dynamic bodies change their
  velocities for the next step. Soft bodies simulate in parallel; rigid velocity changes are merged
  in soft body order, so results do not depend on the worker count.

### Design notes

- Soft body constraints are solved in four wide batches of vertex disjoint constraints, built
  greedily in the settings' order with a 64 constraint window. `optimize()` sorts vertices along a
  Morton curve (collision patches of 32 vertices) and edges by graph distance from the kinematic
  vertices, so one sweep carries corrections outward from the anchors. Alternating sweep direction
  was measured and rejected: it doubled the worst stretch.
- Six DOF rotation limits act on the components of the relative rotation quaternion (exact for a
  single axis), with the same Jacobians the revolute joint uses for its collinearity constraint. Its
  per axis state lives in `World.sixDOFStore`, keeping `JointSim` at 444 bytes.
- Joint springs follow Box3D/Jolt: frequency is relative to the axis' own effective mass. A six DOF
  rotation spring on an anchor far from the center of mass is therefore softer than its hertz.

### Port fixes found on the way

- `createJoint` reset a recycled slot's generation to 1, so stale `JointID`s aliased new joints;
  it now increments the slot's revision as `b3CreateJointInternal` does, and `jointType` /
  `jointIsValid` check the world and generation like `b3Joint_IsValid`.
- Added `World.bodyType(_:)` (`b3Body_GetType`).

### Verification

Debug and release: 451 tests in 121 suites pass (56 new tests across vehicles, soft bodies,
characters, buoyancy, and joints, including finite-difference checks of the dihedral gradients and
serial/parallel equivalence for soft bodies and wheel casts). The unsigned arm64 iOS 26 device
build succeeds. Benchmarks for the new systems are in `PERFORMANCE.md`, section 9.

Remaining Jolt features not covered here at the time: a multithreaded constraint solver, path
constraints, skeletal ragdoll/animation utilities, state save/restore, soft body self collision and
soft–soft contact, and double precision large worlds. All but the last were added later; see
"Multithreaded constraint solver" and "Second gap pass".

## Multithreaded constraint solver — 2026-10-24

The solver follows `b3SolverStage` from the C library. With `workerCount > 1` a step wakes a
persistent `WorkerPool` (`Base/WorkerPool.swift`) for contact and joint preparation, the substeps,
restitution, impulse storage, and body finalization; the workers spin on a stage epoch between
stages and sleep on a semaphore outside the solver.

- **Stages** (`Simulation/SolverStages.swift`): body velocity and position integration, then per
  graph color warm start, push, relax, restitution, and store. A color stage covers the color's
  joint units, convex contacts, and mesh contacts as one index space, scalar or 4-wide as the serial
  solver chooses. Blocks are claimed with a per-block epoch word on its own cache line, so a late
  worker can never run a block of a newer stage.
- **Determinism**: constraints in one color share no dynamic body, so a color gives the same bits
  in any partition. Overflow constraints and vehicles touch arbitrary bodies and run on the calling
  thread between colors in serial order. Results are bit-identical to `workerCount = 1` for every
  worker count, and changing `workerCount` between steps is safe.
- **Preparation** (`Simulation/SolverPrepare.swift`): a serial plan assigns each contact its
  constraint slot and manifold offset (prefix sums), then a parallel pass fills them. Joints are
  flattened across colors and prepared in parallel, then packed into wide batches in parallel.
  Joint prepare functions read bodies through raw pointers (`jointPrepareBody`) instead of the
  arrays, to avoid retain traffic and exclusivity checks across threads.
- **Finalization**: the per-body transform, sleep velocity, move event, inertia, and shape bounds
  run in parallel. Island keep-awake bits and the split candidate go to per-thread data merged with
  the serial tie-break rule. The continuous pass of fast bodies, the bullet list, and
  `markProxyMoved` run afterwards in body order, so the broad-phase move buffer is unchanged.
- **Thread count**: stages are separated by barriers, so the solver uses at most
  `performanceCoreCount` threads (`hw.perflevel0.physicalcpu`); on an M1 a thread on an
  efficiency core made 8 threads slower than 4. `begin` waits until every worker checked in, so the
  first stage is shared instead of mostly run by the caller.
- **Swift rules for stage code**: never borrow a class-property array with
  `withUnsafeMutableBufferPointer` on a worker thread (dynamic exclusivity conflicts); take raw
  views on the calling thread before the stage and keep per-thread results behind pointers.
- The collide pass still uses its own `concurrentPerform` dispatch and is not part of the pool.
  A pre-existing exclusivity bug there (reading `contacts.count` while the buffer was borrowed)
  was fixed.

Verification: debug and release, 454 tests in 122 suites pass. `ParallelSolverTests` compares
hashes for 1/4/8 workers over 90 steps on a pyramid, a joint grid, mixed mesh/convex contacts,
restitution, mixed joint types (revolute, six-DOF, weld, distance), ragdolls, and a scene with
640 sleeping boxes plus fast bodies and bullets fired into static walls; it also checks changing
the worker count between steps and vehicles plus soft bodies with workers. Timings are in
`PERFORMANCE.md`, section 10.

## Second gap pass — 2026-10-24

| Gap | Implementation | Files |
|---|---|---|
| Path constraint | `PathJointDef` / `createPathJoint`: a point on body B follows a cubic Hermite `ConstraintPath` (open or looping; Catmull-Rom convenience initializer) fixed in body A. Perpendicular to the path the point is held as a 2x2 block; along it the joint is free, has friction, a velocity motor, or a soft position motor measured in arc length; open paths stop at their ends. Rotation: free, hinged about the tangent, normal, or binormal (2x2 block), aligned with the path frame, or fixed relative to A (3x3 blocks on the rotation error quaternion). Run time setters for path, motor, targets, friction, rotation mode | `Simulation/PathJoint.swift` |
| State save/restore | `World.saveState()` / `restoreState(_:)` (Jolt's StateRecorder with every state flag). The snapshot deep copies all state that influences later steps, including the broad-phase trees, pair set, contact manifolds, constraint graph bit sets, islands, id pools, joint side stores, vehicles, soft bodies, pending events, and the simulation settings. Restoring repeats the following steps bit for bit, also after bodies, shapes, joints, vehicles, or soft bodies were created or destroyed in between. `CharacterVirtual.saveState()` / `restoreState(_:)` cover characters | `Simulation/WorldState.swift`, `Character/CharacterVirtual.swift` |
| Skeletons and ragdolls | `Skeleton`, `SkeletonPose` (local joint states and model space matrices), `SkeletalAnimation` (keyframe tracks, slerp, looping, weighted blending), `SkeletonMapper` (maps between a detailed and a ragdoll skeleton by joint name; joints between two mapped joints form chains that are turned to meet the mapped end), `RagdollSettings` / `Ragdoll`: one body per joint, six DOF joints as swing-twist (cone), pyramid, hinge, or fixed, joint friction and soft limits, mass ratio stabilization, self collision modes (none, connected, overlapping in the creation pose via filter joints, all), posing, motor driving to an animated pose, kinematic driving, whole body impulses | `Skeleton/` |
| Soft body self and soft–soft contact | Vertex–triangle contacts (vertex–vertex for bodies without faces) found once per step with a sorted cell grid in which each feature is padded by its own vertices' motion and each vertex queries the cells its own motion covers; at most four closest triangles per vertex and body; detection runs in parallel over body pairs. XPBD projection keeps a thickness on the side the vertex started on (always the outside of closed surfaces), shares the correction through the barycentric weights, applies friction, and limits penetration recovery to `contactSpeed`. Self contact skips a vertex's own and edge neighbors' triangles and pairs closer than two average edge lengths in the rest pose. Soft bodies whose swept bounds overlap substep together (at the larger iteration count); their per body work runs on the workers | `SoftBody/SoftBodyContacts.swift`, `SoftBody/SoftBodySolver.swift` |

### Design notes

- Path joint rows are recomputed every solve from the closest point, found by a search that starts at
  the previous fraction's segment and rejects other segments by the bounds of their Bezier control
  polygons. The motion of the closest point along the path is orthogonal to the perpendicular
  rows, so the Jacobians use the frame at the closest point only.
- World snapshots keep contact payloads by contact id: the payload pool never moves its slabs, so
  restored contacts point at valid slots. Shape geometry objects (hulls, meshes, height fields,
  compounds, paths) are shared, not copied; they must not be edited in place between a save and a
  restore. A state only restores into the world that saved it.
- A ragdoll part's body origin and rotation are its joint's model transform, so reading a pose is a
  copy of body transforms. Motor driving sets six DOF rotation position motors with the target
  rotation of frame B relative to frame A taken from the animated pose.
- Soft contact margins use each vertex's speed plus what gravity adds over the step.
  A relative velocity bound was tried and rejected: when one body stops on the ground mid step the
  other keeps falling, contacts were missed, and piles of soft cubes tangled.
- Without a recovery limit, a vertex that started a substep inside the thickness was pushed out in
  one projection, which XPBD turns into velocity; a pile of 27 soft cubes gained energy until it
  exploded after about 240 steps.

### Port fixes found on the way

- The collide pass and the sensor pass allocated a bit set every step and never freed it
  (8 bytes per step each in small scenes, growing with the contact and sensor counts).
- `DynamicTree.proxyCount` is settable internally for restores.

### Verification

Debug and release: 480 tests in 126 suites pass. New tests cover path geometry against brute force
sampling, a sliding bead's constraint force, end stops, friction, a looping velocity motor with the
body turning with the path frame, the position motor, every rotation mode, paths on a moving body,
and slot reuse; rollback after 40 steps repeated twice over 90 steps on a scene with every system
(pyramid with a sensor, ragdolls, revolute, six DOF, and path joints, vehicles, cloth, a bullet, and
a character), rollback across structural changes, rollback with worker threads, and repeated
save/restore cycles; skeleton spaces, animation sampling and blending, mapper chains and reverse
mapping, a falling ragdoll staying connected within limits, motor and kinematic driving, mass
stabilization and collision modes; and a folding strip with and without self collision, cloth on
cloth, crossing ropes, worker count independence, and waking. `ParallelSolverTests` gained a scene
with 192 path joints. Remaining from the Jolt comparison: double precision large worlds (edge contacts were added
later, see "Soft body edge collision").

## Soft body edge collision — 2026-10-25

Vertex–face contacts and vertex planes alone let edges pass through each other and through thin
rigid features: two crossing threads or narrow strips touch only between their vertices, and a rod
thinner than the cloth spacing slips between two vertices. Vertex–face plus edge–edge is the
complete set (two triangle meshes first touch in one or the other), so both are now handled.
`SoftBodyDef.enableEdgeCollision` (default on) and `setSoftBodyEdgeCollision` switch it.

| Part | Implementation | Files |
|---|---|---|
| Edge vs rigid features | Once per step every surface edge (face edges, or the constraint edges of bodies without faces) finds the closest feature of each nearby shape: closed form for spheres and capsules, GJK of the segment against hulls and mesh or height field triangles, compound children recursively. When that point lies strictly inside the edge (an end point is the vertex's job) the edge gets a plane through the feature. Each substep keeps the edge point above its plane, splitting the correction by the barycentric weights, with friction; dynamic bodies receive the edge's momentum change and their planes move with their tracked velocity, including within the substep, so Gauss–Seidel over many edges does not overshoot | `SoftBody/SoftBodyEdgeCollision.swift`, `SoftBody/SoftBodyCollision.swift` |
| Edge vs edge (self and soft–soft) | Edge pairs found with the same per-step grid as the vertex contacts, at most four closest per edge and body; self pairs skip edges sharing a vertex or edge neighbor and pairs close in the rest pose (precomputed once per body). Each substep keeps the closest points of a crossing pair `thickness` apart along the common normal on the side they started on; outside the crossing configuration (an end point closest, or near parallel edges) ropes keep the plain distance and faced surfaces leave the pair to the vertex–face contacts | `SoftBody/SoftBodyContacts.swift` |

### Design notes

- Candidates use swept boxes: from a vertex's position along its velocity for the step, padded by
  a quarter of that travel and what gravity adds, instead of a sphere of the full travel. For a
  falling cloth this cut the candidates several fold.
- A rigid edge whose two vertices already rest on the same shape with normals within about 8
  degrees is skipped: the surface between them is flat at that scale. This removed over 90% of the
  rigid edge contacts in cloth piles.
- Sides. Re-reading a pair's side every step from a pair resting slightly inside the thickness can
  pick the wrong side, and the contact then pushes the surfaces through each other; a stack of six
  cloths kept vertices at 6–19 m/s and detection grew to 900 ms per step. Vertex–face pairs now
  remember their side and use it only when the current reading is ambiguous (closer than half the
  thickness); a clear reading always wins, since on an open surface a vertex can legitimately pass
  round a free border to the other side. Edge–edge pairs cannot use history: their common normal
  flips whenever the edges turn past parallel, which folds do constantly, and a remembered side
  (as a sign or as a world direction) let a folding strip pass through itself. Ambiguous edge pairs
  are skipped for that step instead, leaving them to the vertex–face contacts.
- Self contact exclusion lists (faces per vertex, edges per edge) are built at creation, or when
  self collision or the thickness changes, so the first step has no hitch.

### Verification

Debug and release: 484 tests in 127 suites pass. New tests: a narrow strip dropped across a pinned
narrow strip (no vertex of either lies over the other's triangles) hangs on it with edge collision
and falls 3 m without; two crossing ropes catch each other between their vertices; a cloth with
0.25 m spacing falling on a 1 cm rod between its vertices is caught, and a free light rod is carried
down at the speed momentum conservation gives (about 5.4 m/s); results are bit identical for 1 and
6 workers. The earlier self collision, cloth on cloth, rope, waking, and rollback tests still pass.


## Double precision world coordinates

World positions now use `Pos` (three `Double` components) and `WorldTransform` (`Pos` translation,
`Quat` rotation). `Vec3`, `Transform`, shape geometry, velocities, inertia, contact anchors, joint
Jacobians, and the SIMD solver remain single precision. This is the default representation, with
no build flag. Code that previously assigned a `Vec3` to a world position must explicitly promote
it using `Pos(vector)`; creating `Pos` from `Double` coordinates avoids losing detail before the
engine receives the position.

The precision boundary is subtraction: `subPos(a, b)` subtracts in double before narrowing the
relative displacement. Narrow phase collision, TOI sweeps, shape queries, buoyancy, and constraint
preparation work relative to a nearby body or query origin. World-space query hits, body events,
character positions, skeleton root offsets, pulley anchors, and snapshot positions retain double
precision. `shapeRayCast` now returns `WorldCastOutput`; low-level `CastOutput` continues to contain
a local `Vec3`. `shapeClosestPoint` accepts and returns `Pos`.

Spatial trees keep compact float AABBs. Bounds are inflated locally before translating in double,
then rounded outward. Ray broad phase uses a conservative box cast around the exact world origin;
the narrow phase still casts a zero-origin ray. Tree box casts account for midpoint rounding when
computing extents. `WorldAABB`, `shapeWorldAABB`, `bodyWorldAABB`, and `softBodyWorldAABB` expose
precise bounds; existing AABB accessors expose conservative float bounds. At extreme coordinates
these float boxes can become much wider than the shapes, increasing broad-phase candidates.
Double positions improve local precision, not the precision of very large individual shapes or
constraints spanning enormous distances. Double ULP also remains a limit (about 0.000122 world
units at 10^12).

Soft bodies store a double origin plus float vertex offsets. Initial origins lie on a 1024-unit
grid; contacting soft bodies share one frame before contact detection and projection. Moving groups
rebase when their local center exceeds 1024 times the world length scale. Rebasing moves current,
previous, skinned, and skin-joint positions together. World vertex access materializes `Pos` values;
`withSoftBodyLocalVertexPositions` instead lends the float buffer and its double origin without
allocation. Skin poses accept `[WorldTransform]` (the `[Transform]` convenience overload promotes
existing single precision world poses).

State hashes include all 64 position bits. Snapshots retain both rigid positions and soft origins.
Float-build trajectory goldens therefore change. The mesh, height-field, and compound simulation
fixtures were regenerated with `BOX3D_DOUBLE_PRECISION=ON`; their generators hash all 24 position
bytes. The wave-pile, mesh-drop, query-spawn, and ragdoll sleep/hash goldens use the C double build.
The origin tests remain, alongside tests at positive/negative 10^9 and at 10^12 for fractional
integration, world queries, conservative bounds, rigid/soft contacts, and rollback.

To regenerate the large simulation fixtures independently:

```sh
cmake -S /path/to/box3d -B /tmp/box3d-double -DCMAKE_BUILD_TYPE=Release \
  -DBOX3D_DOUBLE_PRECISION=ON -DBOX3D_SAMPLES=OFF -DBOX3D_UNIT_TESTS=ON
cmake --build /tmp/box3d-double -j 4
python3 Tools/Reference/regenerate_double_simulations.py /path/to/box3d /tmp/box3d-double
```

For camera-relative rendering, keep the camera itself in `Pos` and use
`toRelativeTransform(world.bodyTransform(body), cameraPosition)`; avoid converting an absolute
world position with `toVec3` before subtracting the camera. Fluid volumes also accept `WorldAABB`,
so both the overlap test and the fluid surface height retain their world precision. Soft-to-soft
contact grouping compares precise world bounds directly, preventing float-tree cell coarseness
from combining unrelated pieces. Compact tree endpoints still require coordinates within the
finite float range.

## Collision quality and spatial indexing — 2026-10-26

`isDoublePrecision()` now returns true, matching `Pos` and `WorldTransform`. This reports world
position precision; local geometry, velocities, and the constraint solver continue to use Float.

### Moving-pair CCD

Set `world.enableDynamicContinuous = true` to enable the supplemental convex moving-pair pass.
It is opt-in so the default continues to reproduce the upstream Box3D trajectories. The pass
captures every awake body's start pose before finalization, compares swept **double precision**
bounds, and orders impacts by fraction and stable candidate indices. Fast dynamic bodies can hit
other dynamic bodies, bullets, and kinematic targets even when their endpoint bounds do not overlap.
Rotating hull queries include a COM bounding sphere so intermediate arcs reach static candidates.

The pass conservatively truncates dynamic motion at an accepted impact and resolves its normal
velocity immediately, with both masses, rotational inertia at impact, and restitution. Sleeping
targets receive their impulses after finalization releases its raw array views; newly awakened
islands are explicitly kept awake for the remainder of the step. Hit events from these impacts
can have a null ContactID because the persistent contact may only be created next step. Collision
filters and pre-solve rejection apply. The legacy CCD sensor-hit list no longer drops hits after
the eighth sensor.

This is not a general TOI island solver: it does not integrate the remaining interval through an
arbitrary chain of impacts. Moving triangle meshes and soft-body CCD are outside this pass. Use
substeps for densely coupled high-speed scenes.

### Soft/rigid contact quality

A soft vertex retains up to four independent rigid surface planes instead of only the nearest
plane. Parallel planes from one collider are merged; independent walls and floors remain active
together. The primary plane arrays remain available to the edge detector. Normal velocity removal
only cancels approach, and tangential friction excludes the separating normal velocity.

The staged rigid/soft coupling is unchanged: rigid poses finalize before soft simulation, and soft
reaction impulses affect rigid motion on the next step. A unified rigid/soft substep solver, rigid
motion-aware soft CCD, and shared rigid velocity state across separate soft bodies remain future
work. The four-plane cap is a contact reduction policy, not an intersection-free guarantee.

### Moving compounds

`createDynamicCompound(body, shapeDef, compound)` instantiates a convex-only baked Compound as
fixtures on a dynamic or kinematic body and returns `DynamicCompoundInstance`. Each child has an
ordinary ShapeID; `instance.childIndex(of: hit.shapeId)` maps query hits back to the baked child.
Child transforms and materials are preserved, and mass is updated once after construction.
`replaceDynamicCompound` validates/creates the replacement before removing the old fixtures;
`destroyDynamicCompound` removes remaining live children without disturbing unrelated fixtures.
Instances participate in existing contacts, sensors, terrain collision, CCD, and world rollback.

Mesh children are rejected before any fixture is attached. This API uses one world proxy per child;
the packed single-proxy `createCompoundShape` API remains static-only. A mutable moving compound
with its own shared BVH and compound-to-compound child traversal is still a separate extension.

### Spatial queries and grouping

Soft grouping and soft/soft waking use a sweep along the widest world axis, followed by precise
double AABB rejection. Pair output has canonical input-index order. Sparse scenes avoid the former
unconditional n(n-1)/2 comparisons; fully overlapping scenes still have quadratic output.

Soft-body ray casts lazily build a local face BVH for surfaces with at least 32 triangles. The cache
is keyed by body generation and vertex revision, discarded on destruction/restore, and rebuilt
after simulation, vertex edits, or teleports. Equal-fraction hits choose the lowest face index,
matching the linear scan. `world.enableSoftBodyQueryAcceleration = false` retains the scan path for
workloads with too few rays to amortize rebuilding a deforming surface's tree. Vertex edits and
skinning teleports also refresh bounds immediately, so queries between steps see the edited surface.

These changes do not replace the rigid world's absolute-float broad-phase trees with a partitioned
large-world index. The remote-coordinate candidate inflation described above still applies there.

Verification: debug and release each pass 510 tests in 134 suites; Address Sanitizer passes the
18 tests in the five new suites. Existing C trajectory/contact fixtures are unchanged. New checks
cover reversed creation order, bullet and ordinary moving pairs, rotation-only static impacts,
more than eight sensor hits, restitution with unequal masses and sleeping targets, filters,
rollback, worker parity at 10^12, rigid wall/floor corners, compound mass/materials/lifecycle,
spatial pairs against an exhaustive oracle, and cached rays against the linear scan after edits,
restores, and id reuse. Performance measurements and their limits are in `PERFORMANCE.md`.
