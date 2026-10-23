# Natsel porting notes

Natsel is a Swift port of the Box3D C physics engine. The goal is algorithmic parity with the C
sources (including floating-point operation order) so the determinism hashes from
`test/test_determinism.c` can be reproduced.

## Platform and style

- Minimum deployment target: macOS 26 / iOS 26 (needed for `InlineArray`; `Synchronization`
  atoms need macOS 15).
- Public API is Swift-idiomatic: `Vec3`, `Transform`, `BodyID`, `World`, and free functions such
  as `makeBoxHull` / `computeCosSin` instead of the C `b3*` prefixes.
- Internal C-isms keep their original names (`b3Assert`, `b3Validate`, `b3Alloc`, `hash64NonZero`)
  so the ported code reads against the C sources.

## Determinism rules

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
`WorkerParityTest` does not apply: Natsel solves on one thread, so `workerCount` is always 1 and
determinism is inherent rather than depending on work partitioning.

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
