#include <Shapes.h>
#include <functions/Transform.h>

void updateAABB(Shape *shape)
{
    switch (shape->type)
    {
    case SHAPE_BOX:
        computeBoxAABB(shape);
        break;
    case SHAPE_CIRCLE:
        computeCircleAABB(shape);
        break;
    case SHAPE_POLYGON:
        computePolygonAABB(shape);
        break;
    }
}

void computeBoxAABB(Shape *shape)
{
    BoxShapeData *boxData = (BoxShapeData *)shape->data;
    float w = boxData->width / 2.0f;
    float h = boxData->height / 2.0f;

    // Getting four dots that define our box
    Vector2 points[4] = {
        {-w, -h}, {w, -h}, {w, h}, {-w, h}};

    for (size_t i = 0; i < 4; i++)
    {
        points[i] = transformPoint(shape->transform, points[i]);
    }

    shape->bounds.min = points[0];
    shape->bounds.max = points[0];
    for (size_t i = 1; i < 4; i++)
    {
        if (points[i].x < shape->bounds.min.x)
            shape->bounds.min.x = points[i].x;
        if (points[i].x > shape->bounds.max.x)
            shape->bounds.max.x = points[i].x;
        if (points[i].y > shape->bounds.max.y)
            shape->bounds.max.y = points[i].y;
        if (points[i].y < shape->bounds.min.y)
            shape->bounds.min.y = points[i].y;
    }
}

void computeCircleAABB(Shape *shape)
{
    CircleShapeData *data = (CircleShapeData *)shape->data;
    float r = data->r;

    shape->bounds.max = (Vector2){shape->transform.pos.x + r, shape->transform.pos.y + r};
    shape->bounds.min = (Vector2){shape->transform.pos.x - r, shape->transform.pos.y - r};
}

void computePolygonAABB(Shape *shape)
{
    PolygonShapeData *data = (PolygonShapeData *)shape->data;

    Vector2 tPoints[data->count];

    for (size_t i = 0; i < data->count; i++)
    {
        tPoints[i] = transformPoint(shape->transform, data->points[i]);
    }

    // Searching for min and max points on AABB
    shape->bounds.max = tPoints[0];
    shape->bounds.min = tPoints[0];
    for (size_t i = 1; i < data->count; i++)
    {
        if (tPoints[i].x < shape->bounds.min.x)
            shape->bounds.min.x = tPoints[i].x;
        if (tPoints[i].x > shape->bounds.max.x)
            shape->bounds.max.x = tPoints[i].x;
        if (tPoints[i].y > shape->bounds.max.y)
            shape->bounds.max.y = tPoints[i].y;
        if (tPoints[i].y < shape->bounds.min.y)
            shape->bounds.min.y = tPoints[i].y;
    }
}

Shape createBox(float height, float width, Vector2 pos, float angle)
{
    Shape box;
    box.bounds.max = (Vector2){0, 0};
    box.bounds.min = (Vector2){0, 0};
    box.type = SHAPE_BOX;
    BoxShapeData *boxData = malloc(sizeof(BoxShapeData));
    if (!boxData)
    {
    }
    boxData->height = height;
    boxData->width = width;
    box.data = boxData;
    box.transform.pos = pos;
    box.transform.R = createRotationMatrix(angle);
    computeBoxAABB(&box);
    return box;
}

Shape createCircle(float r, Vector2 pos)
{
    Shape circle;
    circle.bounds.max = (Vector2){0, 0};
    circle.bounds.min = (Vector2){0, 0};
    circle.type = SHAPE_CIRCLE;
    CircleShapeData *circleData = malloc(sizeof(CircleShapeData));
    if (!circleData)
    {
    }
    circleData->r = r;
    circle.data = circleData;
    circle.transform.pos = pos;
    circle.transform.R = createRotationMatrix(0);
    computeCircleAABB(&circle);
    return circle;
}

Shape *createPolygon(Vector2 points[], size_t count, Vector2 pos, float angle)
{
    Shape *polygon = malloc(sizeof(Shape));
    if (!polygon)
        return NULL;

    polygon->type = SHAPE_POLYGON;

    PolygonShapeData *polyData = malloc(sizeof(PolygonShapeData));
    if (!polyData)
    {
        free(polygon);
        return NULL;
    }

    polyData->points = malloc(sizeof(Vector2) * count);
    if (!polyData->points)
    {
        free(polyData);
        free(polygon);
        return NULL;
    }

    for (size_t i = 0; i < count; i++)
    {
        polyData->points[i] = points[i];
    }
    polyData->count = count;

    polygon->data = polyData;
    polygon->transform.pos = pos;
    polygon->transform.R = createRotationMatrix(angle);

    computePolygonAABB(polygon);

    return polygon;
}

Shape *convertBoxToPoly(Shape *box)
{
    BoxShapeData *boxData = (BoxShapeData *)box->data;
    int BOX_POINTS_COUNT = 4;

    Vector2 *points0 = malloc(sizeof(Vector2) * BOX_POINTS_COUNT);

    points0[0] = (Vector2){boxData->width / 2, boxData->height / 2};
    points0[1] = (Vector2){-boxData->width / 2, boxData->height / 2};
    points0[2] = (Vector2){-boxData->width / 2, -boxData->height / 2};
    points0[3] = (Vector2){boxData->width / 2, -boxData->height / 2};
    Shape *result = createPolygon(points0, BOX_POINTS_COUNT, box->transform.pos, getRotationalAngleDeg(box->transform.R));
    free(points0);
    return result;
}

void deleteShape(Shape *shape)
{
    if (shape == NULL || shape->data == NULL)
        return;

    switch (shape->type)
    {
    case SHAPE_BOX:
    case SHAPE_CIRCLE:
        free(shape->data);
        break;

    case SHAPE_POLYGON:
    {
        PolygonShapeData *data = (PolygonShapeData *)shape->data;
        free(data->points);
        free(data);
        break;
    }

    default:
        break;
    }

    shape->data = NULL;
}