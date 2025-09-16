#include <World.h>


World initWorld(int capacity){
    World world;
    world.bodies_count = 0;
    world.capacity = capacity;
    world.bodies = malloc(sizeof(RigidBody *) * capacity);
    return world;
}

void addBody(World *world, RigidBody *body){
    if(world->bodies_count >= world->capacity){
        world->capacity = world->capacity > 0 ? world->capacity * 2 : 16;
        RigidBody **bodies = realloc(world->bodies, sizeof(RigidBody *) * world->capacity);
        if(!bodies){
            printf("\n FAILED TO REALLOC WORLD BODIES \n");
            exit(1);
        }

        world->bodies = bodies;
    }
    world->bodies[world->bodies_count++] = body;
}

void freeWorld(World *w){
    for (size_t i = 0; i < w->bodies_count; i++)
    {
        free(w->bodies[i]);
    }
    free(w->bodies);
    w->bodies = NULL;
    w->bodies_count = 0;
    w->capacity = 0;
}

