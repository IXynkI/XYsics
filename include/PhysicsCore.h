#ifndef PHYSICS_CORE_H
#define PHYSICS_CORE_H

#define M_PI 3.14159265358979323846

#include <Shapes.h>
#include <MathCore.h>
#include <World.h>
#include <structs/PhysicsStructs.h>



RigidBody *createRigidBody(Shape shape, Vector2 linearVelocity, Vector2 force, float restitution, float angularVelocity, float torque, float mass);
void applyForcesAndMove(World *world, float deltaTime);
float calculateInertia(RigidBody *body);
float calculateInertiaPolygon(RigidBody *body);


#endif