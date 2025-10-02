#ifndef PHYSICSSTRUCTS_H
#define PHYSICSSTRUCTS_H

#include <structs/MathStructs.h>
#include <Shapes.h>

typedef enum
{
    RIGID_BODY,
    STATIC_BODY,
    BODY_TYPE_NUM
} BodyType;

typedef struct
{
    Vector2 linearVel;
    Vector2 force;

    float restitution;
    float angularVel;
    float torque;
    float mass;
    float momentOfInertia;
} RigidBodyData;

typedef struct StaticBody
{
    float restitution;
    float mass;
} StaticBodyData;

typedef struct Body
{   
    Shape shape;
    BodyType type;
    void *data;
} Body;

#endif