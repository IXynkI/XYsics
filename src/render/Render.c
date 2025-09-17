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

    float halfW = data->width / 2;
    float halfH = data->height / 2;

    Vector2 points[4] = {
        {-halfW, -halfH}, {halfW, -halfH}, {halfW, halfH}, {-halfW, halfH}};

    
    for (size_t i = 0; i < 4; i++)
    {
        points[i] = transformPoint(shape->transform, points[i]);
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

    transformPoints(shape->transform, data->points, data->count, tPoints);
    
    for (size_t i = 0; i < data->count; i++)
    {
        drawLine(&tPoints[i], &tPoints[(i + 1) % data->count]);
    }
}

void drawCircle(Shape *shape)
{
    if (shape == NULL) {
        printf("Shape is NULL\n");
        return;
    }

    if (shape->data == NULL) {
        printf("Shape data is NULL\n");
        return;
    }
    
    CircleShapeData *data = (CircleShapeData *)shape->data;
    float r = data->r;

    XDrawArc(display, window, gc,
             shape->transform.pos.x - r, shape->transform.pos.y - r,
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