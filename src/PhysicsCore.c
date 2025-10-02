#include <PhysicsCore.h>
#include <Globals.h>

Body *createRigidBody(Shape shape, float restitution, float mass)
{
    Body *body = malloc(sizeof(Body));
    if (!body)
    {
        printf("Error while allocating memory for Body");
        return NULL;
    }

    RigidBodyData *data = malloc(sizeof(RigidBodyData));
    if (!data)
    {
        printf("Error while allocating memory for RigidBodyData");
        free(body);
        return NULL;
    }

    body->type = RIGID_BODY;

    body->shape = shape;
    data->linearVel = (Vector2){0, 0};
    data->force = (Vector2){0, 0};
    data->restitution = restitution;
    data->angularVel = 0.0f;
    data->torque = 0.0f;
    data->mass = mass;
    body->data = data;
    data->momentOfInertia = calculateInertia(body);

    return body;
}

Body *createStaticBody(Shape shape, float restitution)
{
    Body *body = malloc(sizeof(Body));
    if (!body)
    {
        printf("Error while allocating memory for Body");
        return NULL;
    }
    StaticBodyData *data = malloc(sizeof(StaticBodyData));
    if (!data)
    {
        printf("Error while allocating memory for StaticBodyData");
        free(body);
        return NULL;
    }
    data->restitution = restitution;
    data->mass = INFINITY;

    body->shape = shape;
    body->type = STATIC_BODY;
    body->data = data;

    return body;
}

// For now only applies steady movement
void applyForcesAndMove(World *w, float deltaTime)
{
    for (size_t i = 0; i < w->bodies_count; i++)
    {
        // Applying gravity
        Body *body = w->bodies[i];

        if (body->type == STATIC_BODY)
            continue;

        RigidBodyData *data = (RigidBodyData *)body->data;
        Vector2 deltaGravity = {0, GRAVITY * deltaTime};
        data->linearVel = addVectors(&data->linearVel, &deltaGravity);

        // Moving
        body->shape.transform.pos.x += data->linearVel.x * deltaTime;
        body->shape.transform.pos.y += data->linearVel.y * deltaTime;

        float angleRad = data->angularVel * deltaTime * (M_PI / 180.0f);
        float cosTheta = cosf(angleRad);
        float sinTheta = sinf(angleRad);

        Matrix2 R_delta =
            {cosTheta, -sinTheta,
             sinTheta, cosTheta};

        // Multiply R_delta * b->rotation
        Matrix2 R_new;
        R_new.m00 = R_delta.m00 * body->shape.transform.R.m00 + R_delta.m01 * body->shape.transform.R.m10;
        R_new.m01 = R_delta.m00 * body->shape.transform.R.m01 + R_delta.m01 * body->shape.transform.R.m11;
        R_new.m10 = R_delta.m10 * body->shape.transform.R.m00 + R_delta.m11 * body->shape.transform.R.m10;
        R_new.m11 = R_delta.m10 * body->shape.transform.R.m01 + R_delta.m11 * body->shape.transform.R.m11;

        updateAABB(&body->shape);

        body->shape.transform.R = R_new;
    }
}

float calculateInertia(Body *body)
{
    if (body->type == STATIC_BODY)
    {
        return INFINITY;
    }

    RigidBodyData *data = (RigidBodyData *)body->data;
    switch (body->shape.type)
    {
    case SHAPE_BOX:
        BoxShapeData *boxData = (BoxShapeData *)body->shape.data;
        float w = boxData->width;
        float h = boxData->width;
        return (data->mass * (w * w + h * h)) / 12.0f;
        break;
    case SHAPE_CIRCLE:
        CircleShapeData *circleData = (CircleShapeData *)body->shape.data;
        return (0.5f * data->mass * circleData->r * circleData->r);
        break;
    case SHAPE_POLYGON:
        float result = calculateInertiaPolygon(body);
        return result;
        break;
    }
}

float calculateInertiaPolygon(Body *body)
{
    PolygonShapeData *polyData = (PolygonShapeData *)body->shape.data;
    RigidBodyData *data = (RigidBodyData *)body->data;

    float area = 0;
    Vector2 centroid = {0, 0};
    Vector2 centroidSum = {0, 0};
    float inertiaCentroid = 0;
    float inertiaOrigin = 0;
    float inertiaOriginSum = 0;
    for (size_t i = 0; i < polyData->count; i++)
    {
        Vector2 a = polyData->points[i];
        Vector2 b = polyData->points[(i + 1) % polyData->count];

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

    float inertiaFactor = 1.0f / 12.0f;
    inertiaOrigin = inertiaFactor * inertiaOriginSum;

    float density = data->mass / absArea;

    inertiaCentroid = density * inertiaOrigin - data->mass * (powf(centroid.x, 2) + powf(centroid.y, 2));

    return inertiaCentroid;
}

void setLinearVel(Body *b, Vector2 linearVel)
{
    switch (b->type)
    {
    case STATIC_BODY:
        break;

    case RIGID_BODY:
        RigidBodyData *data = (RigidBodyData *)b->data;
        data->linearVel = linearVel;
        break;
    }
}

Vector2 getImpulseAtContact(Vector2 *separationNormal, Vector2 *contactPoint, Vector2 *posA, Vector2 *posB, Vector2 *linearVelA, Vector2 *linearVelB, float angularVelA, float angularVelB, float invInertiaA, float invInertiaB, float sumInvMass, float restitutionA, float restitutionB)
{

    Vector2 rA = substractVectors(contactPoint, posA);
    Vector2 rB = substractVectors(contactPoint, posB);
    Vector2 crossA = crossProductVF(&rA, angularVelA);
    Vector2 crossB = crossProductVF(&rB, angularVelB);

    // Velocity in contact point
    Vector2 velocityA;
    if (linearVelA)
    {
        velocityA = addVectors(linearVelA, &crossA);
    }
    else
    {
        velocityA = (Vector2){0, 0};
    }

    Vector2 velocityB;
    if (linearVelB)
    {
        velocityB = addVectors(linearVelB, &crossB);
    }
    else
    {
        velocityB = (Vector2){0, 0};
    }
    // Relative velocity in contact point
    Vector2 relativeVelocity = substractVectors(&velocityB, &velocityA);

    float velAlongNormal = dotProduct(&relativeVelocity, separationNormal);

    if (velAlongNormal >= 0)
    {
        return (Vector2){0, 0};
    }

    float denom = sumInvMass +
                  (powf(crossProduct(&rA, separationNormal), 2) * invInertiaA) +
                  (powf(crossProduct(&rB, separationNormal), 2) * invInertiaB);

    float bounciness = fminf(restitutionA, restitutionB);
    float j = -(1 + bounciness) * (velAlongNormal) / denom;
    if (j > MAX_IMPULSE)
    {
        j = MAX_IMPULSE;
    }
    else if (j < -MAX_IMPULSE)
    {
        j = -MAX_IMPULSE;
    }
    Vector2 impulseOnContact = multiplyVectorF(separationNormal, j);
    return impulseOnContact;
}

// impulse based on Box2D implementation
Vector2 getImpulseAtContactStatic(Vector2 *separationNormal, Vector2 *contactPoint, Vector2 *posA, Vector2 *posB, Vector2 *linearVelA, float angularVelA, float invInertiaA, float invMassA, float restitutionA, float restitutionB)
{

    
    Vector2 rA = substractVectors(contactPoint, posA);
    Vector2 crossrAang = crossProductVF(&rA, angularVelA);

    Vector2 velAtContactA = addVectors(linearVelA, &crossrAang);
    Vector2 relativeVel = multiplyVectorF(&velAtContactA, -1.0f);

    float velAlongNormal = dotProduct(&relativeVel, separationNormal);

    if (velAlongNormal >= 0.0001f)
    {
        return (Vector2){0, 0};
    }

    // denom Box2D: invMassA + (rA × n)^2 * invInertiaA
    float rnA = crossProduct(&rA, separationNormal);
    float denom = invMassA + (rnA * rnA) * invInertiaA;
    if (denom < 1e-8f)
    {
        return (Vector2){0, 0};
    }
    float bounciness = fminf(restitutionA, restitutionB);
    float j = -(1.0f + bounciness) * velAlongNormal / denom;
    if (j > MAX_IMPULSE)
    {
        j = MAX_IMPULSE;
    }
    else if (j < -MAX_IMPULSE)
    {
        j = -MAX_IMPULSE;
    }

    return multiplyVectorF(separationNormal, j);
}
