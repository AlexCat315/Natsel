// Shared construction inputs for the height-field C test oracles.
static b3HeightFieldData* MakeReferenceHeightField(int scenario) {
    b3Vec3 scale={0.75f,1.25f,0.5f};
    if(scenario<2) return b3CreateGrid(6,7,scale,scenario==1);
    if(scenario<4) return b3CreateWave(6,7,scale,0.2f,0.3f,scenario==3);
    float heights[42]; uint8_t materials[30];
    for(int i=0;i<42;++i) heights[i]=(float)((i*17)%29)*0.125f-1.5f;
    for(int i=0;i<30;++i) materials[i]=i%11==0?255:i%3;
    b3HeightFieldDef def={.heights=heights,.materialIndices=materials,.scale=scale,.countX=7,.countZ=6,
        .globalMinimumHeight=-1,.globalMaximumHeight=1,.clockwiseWinding=scenario==5};
    return b3CreateHeightField(&def);
}
