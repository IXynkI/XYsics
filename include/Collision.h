#ifndef COLLISION_H
#define COLLISION_H

#include <Utils.h>
#include <MathCore.h>
#include <PhysicsCore.h>
#include <structs/MathStructs.h>
#include <structs/PhysicsStructs.h>
#include <World.h>

typedef struct CollisionPair
{
    int idA;
    int idB;
    bool collided;
    float MTVlength;
    Vector2 axis;
    float contactDepth[2];
    Vector2 contactPoints[2]; //Convex shapes can only have 2 points of contact
    int contactCount;         //Real amount of contact points
} CollisionPair;

typedef struct CollisionPairs
{
    CollisionPair *pairs;
    size_t count;
    size_t capacity;
} CollisionPairs;

typedef void (*CollisionResolver)(Body *a, Body *b, CollisionPair *collPair);

extern CollisionResolver collisionTable[BODY_TYPE_NUM][BODY_TYPE_NUM];

void initCollisionTable();

bool TestAABBOverlap(AABB *a, AABB *b);
bool TestPolygonOverlapSAT(Vector2 polygon1[], size_t size1, Vector2 polygon2[], size_t size2, CollisionPair *collPair);

bool checkNarrowBoxCircle(Shape *box, Shape *circle, CollisionPair *collPair);
bool checkNarrowBoxBox(Shape *box0, Shape *box1, CollisionPair *collPair);
bool checkNarrowCircleCircle(Shape *circle0, Shape *circle1, CollisionPair *collPair);
bool checkNarrowPolygonCircle(Shape *polygon, Shape *circle, CollisionPair *collPair);
bool checkNarrowPolygonPolygon(Shape *polygon0, Shape *polygon1, CollisionPair *collPair);
//Temporary function. Don't want to do it now
bool checkNarrowBoxPolygon(Shape *box, Shape *polygon, CollisionPair *collPair);

bool GJK(Vector2 verticies1[], size_t count1, Vector2 verticies2[], size_t count2);

void initCollisionPairs(CollisionPairs *collPairs, size_t capacity);
void addCollisionPair(CollisionPairs *collPairs, CollisionPair newCollisionPair);
void freeCollisionPairs(CollisionPairs *collPairs);

int compareAABB_X(const void *p1, const void *p2);

void resolveCollisionPairs(Body **bodies, CollisionPairs *collPairs);

void sweepAndPrune(Body **bodies, size_t count, CollisionPairs *candidates);

void checkNarrow(Body **bodies, CollisionPairs *collPairs, CollisionPairs *canditates);

void checkAndResolveCollisions(World *w);

//Resolving any possible pair of collision
void resolveRigidRigid(Body *a, Body *b, CollisionPair *collPair);
void resolveRigidStatic(Body *a, Body *b, CollisionPair *collPair);
void resolveStaticStatic(Body *a, Body *b, CollisionPair *collPair);


#endif