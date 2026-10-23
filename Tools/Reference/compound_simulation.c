// Test oracle only; never linked into Natsel.
// clang -O2 -DNDEBUG -ffp-contract=off -I <box3d>/src -I <box3d>/include compound_simulation.c <build>/src/libbox3d.a -o /tmp/compound-simulation
#include "physics_world.h"
#include "contact.h"
#include "compound.h"
#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"
#include <stdio.h>
#include <string.h>

#include "simulation_digest.h"
#include "compound_fixture.h"
int main(void) {
    for(int scenario=0;scenario<108;++scenario) {
        int index=scenario%54;
        b3WorldDef worldDef=b3DefaultWorldDef(); worldDef.enableSleep=index>=36;
        b3WorldId worldId=b3CreateWorld(&worldDef);
        b3BodyDef bodyDef=b3DefaultBodyDef();
        bodyDef.rotation=b3MakeQuatFromAxisAngle((b3Vec3){0,1,0},0.2f);
        bodyDef.position=(b3Pos){0.125f,-0.25f,-0.125f};
        b3BodyId ground=b3CreateBody(worldId,&bodyDef);
        b3CompoundData* data=MakeReferenceCompound((index%18)/3);
        b3ShapeDef shapeDef=b3DefaultShapeDef();
        shapeDef.enableHitEvents=true;
        if(scenario<54) b3CreateBakedCompoundShape(ground,&shapeDef,data);
        bodyDef=b3DefaultBodyDef(); bodyDef.type=b3_dynamicBody;
        bodyDef.position=(b3Pos){-0.625f,2,0.625f}; bodyDef.rotation=b3MakeQuatFromAxisAngle((b3Vec3){0,1,0},0.3f);
        bodyDef.linearVelocity=(b3Vec3){0.35f,-0.1f,0.2f}; bodyDef.angularVelocity=(b3Vec3){0.1f,0.2f,0.15f};
        if(index>=18 && index<36) { bodyDef.linearVelocity.y=-150.0f; bodyDef.isBullet=true; }
        b3BodyId body=b3CreateBody(worldId,&bodyDef);
        shapeDef=b3DefaultShapeDef(); shapeDef.density=1000; shapeDef.baseMaterial.friction=0.7f;
        shapeDef.baseMaterial.restitution=0.15f; shapeDef.baseMaterial.rollingResistance=0.05f;
        if(index%3==0) { b3Sphere sphere={{0,0,0},0.4f}; b3CreateSphereShape(body,&shapeDef,&sphere); }
        if(index%3==1) { b3Capsule capsule={{0,-0.35f,0},{0,0.35f,0},0.3f}; b3CreateCapsuleShape(body,&shapeDef,&capsule); }
        if(index%3==2) { b3BoxHull hull=b3MakeBoxHull(0.4f,0.4f,0.4f); b3CreateHullShape(body,&shapeDef,&hull.base); }
        if(scenario>=54) { b3ShapeDef groundDef=b3DefaultShapeDef(); groundDef.enableHitEvents=true; b3CreateBakedCompoundShape(ground,&groundDef,data); }
        uint64_t bodies=14695981039346656037ull, contacts=bodies;
        for(int step=0;step<180;++step) {
            b3World_Step(worldId,1.0f/60.0f,4);
            b3WorldTransform xf=b3Body_GetTransform(body);
            bodies=Vector(bodies,xf.p); bodies=Vector(bodies,xf.q.v); bodies=Scalar(bodies,xf.q.s);
            bodies=Vector(bodies,b3Body_GetLinearVelocity(body)); bodies=Vector(bodies,b3Body_GetAngularVelocity(body));
            contacts=Contacts(contacts,b3GetWorldFromId(worldId));
            if((step+1)%30==0) printf("scenario %d step %d bodies 0x%016llx contacts 0x%016llx\n",scenario,step+1,(unsigned long long)bodies,(unsigned long long)contacts);
        }
        b3DestroyWorld(worldId); b3DestroyCompound(data);
    }
}
