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
Camera camera;
float windowW;
float windowH;

int main()
{
    initCollisionTable();
    Vector2 camPos = {400,400};
    moveAndZoomCamera(&camera, camPos, 3);
    
    Vector2 pos = {150, 800};
    Vector2 pos2 = {900, 400};

    Vector2 linearVel = {7, -30};
    Vector2 force = {0, 0};

    float restitution = 1.0f;
    float angularVelocity = 0.0f;
    float torque = 0.0f;
    float mass = 10.0f;

    world = initWorld(BODIES_COUNT);
    //Box
    Shape shape0 = createBox(2000, 10, pos, 0);
    Body *body0 = createStaticBody(shape0, 0.25f);
    addBody(&world, body0);

    //Circle moving towards box
    Vector2 linearVel2 = {-100, -35};
    Shape shape1 = createCircle(10.0f, pos2);
    Body *body1 = createRigidBody(shape1, restitution, mass);
    addBody(&world, body1);
    setLinearVel(body1, linearVel2);
    
    //Triangle
    Vector2 points[3];
    for (size_t i = 0; i < 3; i++)
    {
        if (i % 2 == 0)
        {
            points[i] = (Vector2){i * 20, 10 + i * 10};
        }
        else
        {
            points[i] = (Vector2){i * 10, 10 - i * 10};
        }
    }


    Shape *shape2 = createPolygon(points, 3, pos, 0);
    Body *body2 = createRigidBody(*shape2, restitution, mass);
    //addBody(&world, body2);

    
    if (!windowInit(2000, 1000, "test"))
    {
        return -1;
    }
    unsigned int white = getColorCode(COLOR_WHITE);
    unsigned int black = getColorCode(COLOR_BLACK);
    setGCStyle(white, black, black, black);
    windowRun(tick);

    return 0;
}
