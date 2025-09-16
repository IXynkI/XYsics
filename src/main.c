#include <Collision.h>
#include <PhysicsCore.h>
#include <Shapes.h>
#include <Utils.h>
#include <stdio.h>
#include <render/Render.h>
#include <Globals.h>
#include <render/Colors.h>

const BODIES_COUNT = 3;
World world;

int main()
{

    Vector2 pos = {150, 400};
    Vector2 pos2 = {900, 400};

    Vector2 linearVel = {7, -30};
    Vector2 force = {0, 0};

    world = initWorld(BODIES_COUNT);
    Shape shape0 = createBox(100, 100, pos, 90);
    RigidBody *body0 = createRigidBody(shape0, linearVel, force, 0, 0, 0, 0, 1, 1);
    addBody(&world, body0);

    /*
        Shape shape1 = createCircle(10, pos2);
    RigidBody *body1 = createRigidBody(shape1, linearVel, force, 0, 0, 3, 0, 1, 1);
    addBody(&world, body1);

    Vector2 points[3];
    for (size_t i = 0; i < 3; i++)
    {
        if (i % 2 == 0)
        {
            points[i] = (Vector2){i * 10, 10 + i * 10};
        }
        else
        {
            points[i] = (Vector2){i * 10, 10 - i * 10};
        }
    }

    Shape shape2 = createPolygon(points, 3, pos, 0);
    RigidBody *body2 = createRigidBody(shape2, linearVel, force, 0, 0, 3, 0, 1, 1);
    addBody(&world, body2);

    */
    
    if (!windowInit(1280, 720, "test"))
    {
        return -1;
    }
    unsigned int white = getColorCode(COLOR_WHITE);
    unsigned int black = getColorCode(COLOR_BLACK);

    Matrix2 testM = createRotationMatrix(90);
    printf("\nRotation Matrix of 90 degrees m00: %f, m01: %f, m10: %f, m11: %f \n", testM.m00, testM.m01, testM.m10, testM.m11);

    setGCStyle(white, black, black, black);
    windowRun(tick);

    return 0;
}
