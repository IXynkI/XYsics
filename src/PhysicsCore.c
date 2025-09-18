#include <PhysicsCore.h>
#include <Globals.h>

RigidBody *createRigidBody(Shape shape, Vector2 linearVelocity, Vector2 force, float restitution, float angularVelocity, float torque, float mass)
{
    RigidBody *body = malloc(sizeof(RigidBody));
    if (!body)
        return NULL;

    body->shape = shape;
    body->linearVel = linearVelocity;
    body->force = force;
    body->restitution = restitution;
    body->angularVel = angularVelocity;
    body->torque = torque;
    body->mass = mass;
    body->momentOfInertia = calculateInertia(body);

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

float calculateInertia(RigidBody *body)
{
    switch (body->shape.type)
    {
    case SHAPE_BOX:
        BoxShapeData *boxData = (BoxShapeData *)body->shape.data;
        float w = boxData->width;
        float h = boxData->width;
        return (body->mass * (w * w + h * h)) / 12.0f;
        break;
    case SHAPE_CIRCLE:
        CircleShapeData *circleData = (CircleShapeData *)body->shape.data;
        return (0.5f * body->mass * circleData->r * circleData->r);
        break;
    case SHAPE_POLYGON:
        float result = calculateInertiaPolygon(body);
        return result;
        break;
    }
}

float calculateInertiaPolygon(RigidBody *body)
{
    PolygonShapeData *data = (PolygonShapeData *)body->shape.data;

    float area = 0;
    Vector2 centroid = {0, 0};
    Vector2 centroidSum = {0, 0};
    float inertiaCentroid = 0;
    float inertiaOrigin = 0;
    float inertiaOriginSum = 0;
    for (size_t i = 0; i < data->count; i++)
    {
        Vector2 a = data->points[i];
        Vector2 b = data->points[(i + 1) % data->count];

        float cross = crossProduct(&a, &b);
        area += cross;

        centroidSum.x += (a.x + b.x) * cross;
        centroidSum.y += (a.y + b.y) * cross;

        inertiaOriginSum += cross * (powf(a.x, 2) + a.x * b.x + powf(b.x, 2) + powf(a.y, 2) + a.y * b.y + powf(b.y, 2));
    }
    area *= 0.5f;
    float absArea = fabsf(area);
    float centroidFactor = 1.0f / (6.0f * area);
    
    centroid = multiplyVectorF(&centroidSum, centroidFactor);

    float inertiaFactor = 1.0f/12.0f;
    inertiaOrigin = inertiaFactor * inertiaOriginSum;

    float density = body->mass / absArea;

    inertiaCentroid = density * inertiaOrigin - body->mass * (powf(centroid.x, 2) + powf(centroid.y, 2));


    return inertiaCentroid;
}