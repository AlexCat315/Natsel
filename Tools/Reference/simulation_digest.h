// Shared field-by-field digest for C simulation reference fixtures.
static uint64_t Word(uint64_t hash, uint32_t word) {
    for(int i=0;i<4;++i) { hash^=(word>>(8*i))&255; hash*=1099511628211ull; } return hash;
}
static uint64_t Scalar(uint64_t hash,float value) { uint32_t word; memcpy(&word,&value,4); return Word(hash,word); }
static uint64_t Vector(uint64_t hash,b3Vec3 value) { return Scalar(Scalar(Scalar(hash,value.x),value.y),value.z); }
static uint64_t Contacts(uint64_t hash,b3World* world) {
    int count=0;
    for(int i=0;i<world->contacts.count;++i) {
        b3Contact* c=world->contacts.data+i;
        if(c->contactId==B3_NULL_INDEX) continue;
        ++count;
        hash=Word(hash,c->flags); hash=Word(hash,c->manifoldCount);
        hash=Scalar(hash,c->friction); hash=Scalar(hash,c->restitution); hash=Scalar(hash,c->rollingResistance);
        hash=Vector(hash,c->tangentVelocity);
        for(int m=0;m<c->manifoldCount;++m) {
            b3Manifold* manifold=c->manifolds+m;
            hash=Vector(hash,manifold->normal); hash=Word(hash,manifold->pointCount);
            hash=Vector(hash,manifold->frictionImpulse); hash=Vector(hash,manifold->rollingImpulse); hash=Scalar(hash,manifold->twistImpulse);
            for(int p=0;p<manifold->pointCount;++p) {
                b3ManifoldPoint* point=manifold->points+p;
                hash=Vector(hash,point->anchorA); hash=Vector(hash,point->anchorB); hash=Scalar(hash,point->separation);
                hash=Scalar(hash,point->normalImpulse); hash=Scalar(hash,point->totalNormalImpulse);
                hash=Word(hash,point->featureId); hash=Word(hash,point->triangleIndex); hash=Word(hash,point->persisted);
            }
        }
    }
    return Word(hash,count);
}
