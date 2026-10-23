// Shared mixed query scene for C test oracles.
typedef struct QueryScene { b3WorldId world; b3Pos origin; b3MeshData* mesh; b3HeightFieldData* height; b3CompoundData* compound; } QueryScene;
static QueryScene MakeQueryScene(int scenario,bool sensors) {
  b3WorldDef wd=b3DefaultWorldDef();b3WorldId world=b3CreateWorld(&wd);
  b3Pos origin={scenario?1000000.0f:0,scenario?-1000000.0f:0,scenario?2000000.0f:0};
  b3MeshData* mesh=b3CreateGridMesh(4,5,0.5f,3,true);b3HeightFieldData* height=MakeReferenceHeightField(4);b3CompoundData* compound=MakeReferenceCompound(4);
  for(int j=0;j<6;++j) {
   b3BodyDef bd=b3DefaultBodyDef();bd.type=sensors?b3_staticBody:j==0?b3_dynamicBody:j==1?b3_kinematicBody:b3_staticBody;
   bd.position=b3OffsetPos(origin,(b3Vec3){(float)(j%3)*0.75f-1.0f,0,0});bd.rotation=b3MakeQuatFromAxisAngle((b3Vec3){0,1,0},(float)j*0.15f);
   b3BodyId body=b3CreateBody(world,&bd);b3ShapeDef sd=b3DefaultShapeDef();sd.filter.categoryBits=1ull<<(j%3);sd.isSensor=sensors;sd.enableSensorEvents=true;
   b3SurfaceMaterial materials[3]={b3DefaultSurfaceMaterial(),b3DefaultSurfaceMaterial(),b3DefaultSurfaceMaterial()};sd.materials=materials;sd.materialCount=3;
   if(j==0) { b3Sphere sphere={{0,0.25f,0},0.6f};b3CreateSphereShape(body,&sd,&sphere); }
   if(j==1) { b3Capsule capsule={{0,-0.5f,0},{0,0.5f,0},0.3f};b3CreateCapsuleShape(body,&sd,&capsule); }
   if(j==2) { b3BoxHull hull=b3MakeBoxHull(0.5f,0.25f,0.75f);b3CreateHullShape(body,&sd,&hull.base); }
   if(j==3) b3CreateMeshShape(body,&sd,mesh,(b3Vec3){1,1,1});
   if(j==4) b3CreateHeightFieldShape(body,&sd,height);
   if(j==5) b3CreateBakedCompoundShape(body,&sd,compound);
  }
  return (QueryScene){world,origin,mesh,height,compound};
}
static void DestroyQueryScene(QueryScene scene) { b3DestroyWorld(scene.world);b3DestroyMesh(scene.mesh);b3DestroyHeightField(scene.height);b3DestroyCompound(scene.compound); }
