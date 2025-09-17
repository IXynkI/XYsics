#include <PhysicsCore.h>
#include <Globals.h>

RigidBody *createRigidBody(Shape shape, Vector2 linearVelocity, Vector2 force, float restitution, float angle, float angularVelocity, float torque, float mass, float momentOfInertia)
{
    RigidBody *body = malloc(sizeof(RigidBody));
    if (!body)
        return NULL;

    body->shape = shape;
    body->linearVel = linearVelocity;
    body->force = force;
    body->restitution = restitution;
    body->angle = angle;
    body->angularVel = angularVelocity;
    body->torque = torque;
    body->mass = mass;
    body->momentOfInertia = momentOfInertia;

    return body;
}

// For now only applies steady movement
void applyForcesAndMove(World *w, float deltaTime)
{
    // Applying gravity
    for (size_t i = 0; i < w->bodies_count; i++)
    {
        RigidBody *b = w->bodies[i];
        Vector2 deltaGravity = {0, GRAVITY * deltaTime};
        b->linearVel = addVectors(&b->linearVel, &deltaGravity);
    }

    // Moving
    for (size_t i = 0; i < w->bodies_count; i++)
    {
        RigidBody *b = w->bodies[i];
        b->shape.transform.pos.x += b->linearVel.x * deltaTime;
        b->shape.transform.pos.y += b->linearVel.y * deltaTime;

        float angleRad = b->angularVel * deltaTime * (M_PI / 180.0f);
        float cosTheta = cosf(angleRad);
        float sinTheta = sinf(angleRad);

        Matrix2 R_delta = 
            {cosTheta, -sinTheta,
            sinTheta, cosTheta};

        // Multiply R_delta * b->rotation
        Matrix2 R_new;
        R_new.m00 = R_delta.m00 * b->shape.transform.R.m00 + R_delta.m01 * b->shape.transform.R.m10;
        R_new.m01 = R_delta.m00 * b->shape.transform.R.m01 + R_delta.m01 * b->shape.transform.R.m11;
        R_new.m10 = R_delta.m10 * b->shape.transform.R.m00 + R_delta.m11 * b->shape.transform.R.m10;
        R_new.m11 = R_delta.m10 * b->shape.transform.R.m01 + R_delta.m11 * b->shape.transform.R.m11;
        
        updateAABB(&b->shape);
        
        b->shape.transform.R = R_new;
    }
}