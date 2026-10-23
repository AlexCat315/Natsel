// Shared construction inputs for compound test oracles.
static b3CompoundData* MakeReferenceCompound(int scenario) {
    int capsuleCounts[]={0,2,0,0,2,6}, hullCounts[]={0,0,2,0,2,6};
    int meshCounts[]={0,0,0,2,2,4}, sphereCounts[]={1,0,0,0,3,8};
    b3SurfaceMaterial materials[3]={b3DefaultSurfaceMaterial(),b3DefaultSurfaceMaterial(),b3DefaultSurfaceMaterial()};
    materials[0].friction=0.4f; materials[0].restitution=0.1f; materials[0].rollingResistance=0.05f;
    materials[0].tangentVelocity=(b3Vec3){0.125f,0,0.25f}; materials[0].userMaterialId=5;
    materials[1]=materials[0]; materials[1].friction=0.7f; materials[1].userMaterialId=6;
    materials[2]=materials[0];
    b3BoxHull hulls[]={b3MakeBoxHull(0.5f,0.25f,0.75f),b3MakeBoxHull(0.4f,0.35f,0.6f)};
    b3MeshData* mesh=b3CreateGridMesh(2,3,0.5f,3,true);
    b3CompoundCapsuleDef capsules[8]={0}; b3CompoundHullDef hullDefs[8]={0};
    b3CompoundMeshDef meshes[8]={0}; b3CompoundSphereDef spheres[8]={0};
    for(int i=0;i<capsuleCounts[scenario];++i) {
        capsules[i].capsule=(b3Capsule){{(float)i-1.0f,-0.5f,0.25f},{(float)i-1.0f,0.5f,0.25f},0.2f};
        capsules[i].material=materials[i%2];
    }
    for(int i=0;i<hullCounts[scenario];++i) {
        hullDefs[i].hull=&hulls[i%3==2?1:0].base;
        hullDefs[i].transform=(b3Transform){{(float)i*0.75f-1.0f,0.25f,-0.5f},b3MakeQuatFromAxisAngle((b3Vec3){0,1,0},(float)i*0.1f)};
        hullDefs[i].material=materials[i%2];
    }
    for(int i=0;i<meshCounts[scenario];++i) {
        meshes[i].meshData=mesh; meshes[i].materials=materials; meshes[i].materialCount=3;
        meshes[i].transform=(b3Transform){{(float)i*1.5f,-0.5f,0.5f},b3MakeQuatFromAxisAngle((b3Vec3){0,1,0},(float)i*0.1f)};
        meshes[i].scale=(b3Vec3){i%2?-1.0f:1.0f,1,1.25f};
    }
    for(int i=0;i<sphereCounts[scenario];++i) {
        spheres[i].sphere=(b3Sphere){{(float)i*0.5f-1.0f,0.25f,1.0f},0.3f};
        spheres[i].material=materials[i%2];
    }
    b3CompoundDef def={.capsules=capsules,.capsuleCount=capsuleCounts[scenario],.hulls=hullDefs,.hullCount=hullCounts[scenario],
        .meshes=meshes,.meshCount=meshCounts[scenario],.spheres=spheres,.sphereCount=sphereCounts[scenario]};
    b3CompoundData* result=b3CreateCompound(&def); b3DestroyMesh(mesh); return result;
}
