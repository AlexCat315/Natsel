// Compile against a separately built Box3D Release library; never link this into Natsel.
// Define NATSEL_BOX3D_BENCHMARK_MAIN as the quoted path to Box3D's benchmark/main.c and supply
// Box3D's include/, src/, shared/, and generated build/src/ include directories. Link libbox3d.a
// and libshared.a. Use -DBOX3D_INTERNAL_BENCHMARKS -O3 -DNDEBUG -ffp-contract=off, matching upstream.
// Only world destruction is wrapped, so hashing happens outside the upstream timed interval.

#include "box3d/box3d.h"
#include "physics_world.h"
#include "recording.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

static void DestroyWorldAndHash(b3WorldId worldId)
{
    if (getenv("NATSEL_HASH") != NULL)
    {
        printf("state: %" PRIx64 "\n", b3HashWorldState(b3GetWorldFromId(worldId)));
    }
    b3DestroyWorld(worldId);
}

#define b3DestroyWorld DestroyWorldAndHash
#include NATSEL_BOX3D_BENCHMARK_MAIN
