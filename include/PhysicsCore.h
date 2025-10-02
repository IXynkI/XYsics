#ifndef PHYSICS_CORE_H
#define PHYSICS_CORE_H

#define M_PI 3.14159265358979323846

#include <Shapes.h>
#include <MathCore.h>
#include <World.h>
#include <structs/PhysicsStructs.h>

Body *createRigidBody(Shape shape, float restitution, float mass);
Body *createStaticBody(Shape shape, float restitution);
void applyForcesAndMove(World *world, float deltaTime);
float calculateInertia(Body *body);
float calculateInertiaPolygon(Body *body);

void setLinearVel(Body *b, Vector2 linearVel);

//чiназес) (13 arguments goes hard)
Vector2 getImpulseAtContact(Vector2 *separationNormal, Vector2 *contactPoint, Vector2 *posA, Vector2 *posB, Vector2 *linearVelA, Vector2 *linearVelB, float angularVelA, float angularVelB, float invInertiaA, float invInertiaB, float sumInvMass, float restitutionA, float restitutionB);
Vector2 getImpulseAtContactStatic(Vector2 *separationNormal, Vector2 *contactPoint, Vector2 *posA, Vector2 *posB, Vector2 *linearVelA, float angularVelA, float invInertiaA, float invMassA, float restitutionA, float restitutionB);


#endif