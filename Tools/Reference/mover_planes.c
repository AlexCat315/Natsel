// Test oracle only; never linked into Natsel.
#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_functions.h"
#include <stdio.h>
#include <string.h>
static uint64_t Word(uint64_t h,uint32_t v) { for(int j=0;j<4;++j) { h^=(v>>(j*8))&255; h*=1099511628211ull; } return h; }
static uint64_t Scalar(uint64_t h,float v) { uint32_t b; memcpy(&b,&v,4); return Word(h,b); }
static uint64_t Vector(uint64_t h,b3Vec3 v) { return Scalar(Scalar(Scalar(h,v.x),v.y),v.z); }
int main(void) {
 uint64_t hash=14695981039346656037ull;
 for(int i=0;i<1000;++i) {
  b3CollisionPlane planes[7]={0}; int count=i%8;
  for(int j=0;j<count;++j) {
   b3Vec3 n={(float)((i+j*3)%9-4),(float)((i*3+j*5)%11-5),(float)((i*7+j*2)%13-6)};
   if(b3LengthSquared(n)==0) n.y=1;
   planes[j].plane.normal=b3Normalize(n); planes[j].plane.offset=(float)((i+j)%17-8)*0.2f;
   planes[j].pushLimit=j%3==0?0.15f:j%3==1?FLT_MAX:0;
   planes[j].push=3;planes[j].clipVelocity=(i+j)%2==0;
  }
  b3Vec3 target={(float)(i%7-3)*0.75f,(float)(i%11-5)*0.25f,(float)(i%13-6)*0.5f};
  b3PlaneSolverResult r=b3SolvePlanes(target,planes,count);
  hash=Vector(hash,r.delta);hash=Word(hash,r.iterationCount);
  for(int j=0;j<count;++j) hash=Scalar(hash,planes[j].push);
  hash=Vector(hash,b3ClipVector(target,planes,count));
 }
 printf("0x%016llx\n",(unsigned long long)hash);
}
