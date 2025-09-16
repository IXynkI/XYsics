#ifndef WORLD_H
#define WORLD_H

#include <structs/PhysicsStructs.h>

typedef struct World
{
    RigidBody **bodies;
    size_t bodies_count;
    size_t capacity;
} World;

World initWorld(int capacity);
void addBody(World *world, RigidBody *body);
void freeWorld(World *w);

#endif