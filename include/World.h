#ifndef WORLD_H
#define WORLD_H

#include <structs/PhysicsStructs.h>
#include <stdio.h>

typedef struct World
{
    Body **bodies;
    size_t bodies_count;
    size_t capacity;
} World;

World initWorld(int capacity);
void addBody(World *world, Body *body);
void freeWorld(World *w);

#endif