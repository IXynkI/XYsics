#include <render/Render.h>
#include <PhysicsCore.h>
#include <Collision.h>

void setGCStyle(unsigned int background, unsigned int foreground, unsigned int line, unsigned int fill)
{
    XSetBackground(display, gc, background);
    XSetForeground(display, gc, foreground);
}

void tick(Display *display, Window window, GC gc, float deltaTime)
{
    beginRender();
    checkAndResolveCollisions(&world);
    applyForcesAndMove(&world, deltaTime);
    drawWholeScene();
    stopRender();
}

void drawWholeScene()
{
    for (size_t i = 0; i < world.bodies_count; i++)
    {
        drawShape(&world.bodies[i]->shape);
        //drawAABB(&world.bodies[i]->shape);
    }
}

void drawShape(Shape *shape)
{
    switch (shape->type)
    {
    case SHAPE_BOX:
        drawBox(shape);
        break;
    case SHAPE_CIRCLE:
        drawCircle(shape);
        break;
    case SHAPE_POLYGON:
        drawPolygon(shape);
        break;
    default:
        break;
    }
}
void drawAABB(Shape *shape)
{
    Vector2 points[4];
    points[0] = shape->bounds.max;
    points[1] = (Vector2){shape->bounds.min.x, shape->bounds.max.y};
    points[2] = (Vector2){shape->bounds.min.x, shape->bounds.min.y};
    points[3] = (Vector2){shape->bounds.max.x, shape->bounds.min.y};
    for (size_t i = 0; i < 4; i++)
    {
        points[i].x -= camera.pos.x;
        points[i].y -= camera.pos.y;
        points[i].x *= camera.zoom;
        points[i].y *= camera.zoom;
        points[i].x += windowW/2;
        points[i].y += windowH/2;
    }
    for (size_t i = 0; i < 4; i++)
    {
        drawLine(&points[i], &points[(i + 1) % 4]);
    }
}
void drawPoint(Vector2 *p)
{
    XDrawPoint(display, window, gc, p->x, p->y);
}
void drawLine(Vector2 *p1, Vector2 *p2)
{
    XDrawLine(display, window, gc, p1->x, p1->y, p2->x, p2->y);
}

void drawBox(Shape *shape)
{
    BoxShapeData *data = (BoxShapeData *)shape->data;
    Matrix2 *m = &shape->transform.R;

    /*
        printf("Matrix:\n");
    printf("[ %f  %f ]\n", m->m00, m->m01);
    printf("[ %f  %f ]\n", m->m10, m->m11);

    */

    float halfW = (data->width / 2);
    float halfH = (data->height / 2);

    Vector2 points[4] = {
        {-halfW, -halfH}, {halfW, -halfH}, {halfW, halfH}, {-halfW, halfH}};

    for (size_t i = 0; i < 4; i++)
    {
        points[i] = transformPoint(shape->transform, points[i]);
        
        points[i].x -= camera.pos.x;
        points[i].y -= camera.pos.y;
        points[i].x *= camera.zoom;
        points[i].y *= camera.zoom;
        points[i].x += windowW/2;
        points[i].y += windowH/2;
    }

    for (size_t i = 0; i < 4; i++)
    {
        drawLine(&points[i], &points[(i + 1) % 4]);
    }
}

void drawPolygon(Shape *shape)
{
    PolygonShapeData *data = (PolygonShapeData *)shape->data;
    Vector2 tPoints[data->count];

    for (size_t i = 0; i < data->count; i++)
    {
        tPoints[i] = transformPoint(shape->transform, data->points[i]);

        tPoints[i].x -= camera.pos.x;
        tPoints[i].y -= camera.pos.y;
        tPoints[i].x *= camera.zoom;
        tPoints[i].y *= camera.zoom;
        tPoints[i].x += windowW/2;
        tPoints[i].y += windowH/2;
    }
    

    for (size_t i = 0; i < data->count; i++)
    {
        drawLine(&tPoints[i], &tPoints[(i + 1) % data->count]);
    }
}

void drawCircle(Shape *shape)
{
    CircleShapeData *data = (CircleShapeData *)shape->data;
    float r = data->r;
    r *= camera.zoom;

    Vector2 screenPos = {0,0};
    screenPos.x += shape->transform.pos.x - camera.pos.x;
    screenPos.y += shape->transform.pos.y - camera.pos.y;
    screenPos.x *= camera.zoom;
    screenPos.y *= camera.zoom;
    screenPos.x += windowW/2;
    screenPos.y += windowH/2;

    XDrawArc(display, window, gc,
             screenPos.x - r, screenPos.y - r,
             r * 2, r * 2,
             0, 360 * 64);
}

void beginRender()
{
    XClearWindow(display, window);
}
void stopRender()
{
    XFlush(display);
}