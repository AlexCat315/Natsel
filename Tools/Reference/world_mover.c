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
typedef struct Capture { uint64_t hash; int count,limit; } Capture;
static bool Planes(b3ShapeId id,const b3PlaneResult* planes,int count,void* context) {
 Capture* c=context;c->hash=Word(c->hash,id.index1);c->hash=Word(c->hash,count);
 for(int i=0;i<count;++i) { b3PlaneResult p=planes[i]; c->hash=Vector(c->hash,p.plane.normal);c->hash=Scalar(c->hash,p.plane.offset);c->hash=Vector(c->hash,p.point);c->hash=Word(c->hash,p.triangleIndex);c->hash=Word(c->hash,p.childIndex);c->hash=Word(c->hash,p.materialIndex); }
 return ++c->count<c->limit;
}
static bool Filter(b3ShapeId id,void* context) { Capture* c=context;c->hash=Word(c->hash,id.index1);return id.index1%2==0; }
int main(void) {
 for(int scenario=0;scenario<2;++scenario) {
  QueryScene scene=MakeQueryScene(scenario,false);b3WorldId world=scene.world;b3Pos origin=scene.origin;
  Capture planes={14695981039346656037ull,0,0},filter={14695981039346656037ull,0,0};uint64_t casts=planes.hash;
  for(int i=0;i<120;++i) {
   float x=(float)((i*7)%19)*0.25f-1,z=(float)((i*11)%17)*0.25f-0.75f,y=(float)(i%9)*0.25f-0.5f;
   b3Capsule mover={{x,y,z},{x+0.1f,y+0.5f,z-0.15f},(float)(i%3)*0.2f};
   b3QueryFilter q=b3DefaultQueryFilter();q.maskBits=i%3==0?1:~0ull;
   planes.count=0;planes.limit=i%4==0?1:100000;b3World_CollideMover(world,origin,&mover,q,Planes,&planes);planes.hash=Word(planes.hash,planes.count);
   mover.center1.y=2;mover.center2.y=2.5f;
   b3Vec3 translation={(float)(i%4-2)*1.25f,-4,(float)(i%5-2)*0.75f};
   float fraction=b3World_CastMover(world,origin,&mover,translation,q,i%2==0?Filter:NULL,&filter);casts=Scalar(casts,fraction);
  }
  printf("%d 0x%016llx 0x%016llx 0x%016llx\n",scenario,(unsigned long long)planes.hash,(unsigned long long)casts,(unsigned long long)filter.hash);
  DestroyQueryScene(scene);
 }
}
