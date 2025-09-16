#ifndef PHYSICSSTRUCTS_H
#define PHYSICSSTRUCTS_H

#include <structs/MathStructs.h>
#include <Shapes.h>

typedef struct
{
    Shape shape;

    Vector2 linearVel;
    Vector2 force;

    float restitution;
    float angle;
    float angularVel;
    float torque;
    float mass;
    float momentOfInertia;
} RigidBody;

#endif