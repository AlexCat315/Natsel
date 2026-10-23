// Test oracle only; never linked into Natsel.
#include "physics_world.h"
#include "contact.h"
#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"
#include "compound.h"
#include <stdio.h>
#include <string.h>
#include "simulation_digest.h"
#include "compound_fixture.h"
#include "height_fixture.h"
#include "query_fixture.h"
int main(void) {
 for(int scenario=0;scenario<2;++scenario) {
  QueryScene scene=MakeQueryScene(scenario,true);b3WorldId world=scene.world;
  b3BodyDef bd=b3DefaultBodyDef();bd.type=b3_kinematicBody;b3BodyId visitor=b3CreateBody(world,&bd);
  b3ShapeDef sd=b3DefaultShapeDef();sd.enableSensorEvents=true;b3Sphere sphere={{0,0,0},0.3f};b3CreateSphereShape(visitor,&sd,&sphere);
  uint64_t hash=14695981039346656037ull;
  for(int i=0;i<120;++i) {
   b3Vec3 p={(float)((i*7)%19)*0.25f-1,(float)(i%9)*0.25f-0.5f,(float)((i*11)%17)*0.25f-0.75f};
   b3Body_SetTransform(visitor,b3OffsetPos(scene.origin,p),b3Quat_identity);
   b3World_Step(world,1.0f/60.0f,4);b3SensorEvents events=b3World_GetSensorEvents(world);
   hash=Word(hash,events.beginCount);hash=Word(hash,events.endCount);
   for(int j=0;j<events.beginCount;++j) { hash=Word(hash,events.beginEvents[j].sensorShapeId.index1);hash=Word(hash,events.beginEvents[j].visitorShapeId.index1); }
   for(int j=0;j<events.endCount;++j) { hash=Word(hash,events.endEvents[j].sensorShapeId.index1);hash=Word(hash,events.endEvents[j].visitorShapeId.index1); }
  }
  printf("%d 0x%016llx\n",scenario,(unsigned long long)hash);DestroyQueryScene(scene);
 }
}
