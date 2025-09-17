#ifndef SHAPES_H
#define SHAPES_H

#include <structs/MathStructs.h>
#include <stddef.h>
#include <stdlib.h>


struct AABB;
struct Vector2;
struct Transform;

typedef enum {
    SHAPE_BOX,
    SHAPE_CIRCLE,
    SHAPE_POLYGON
} ShapeType;

typedef struct Shape
{
    ShapeType type;
    Transform transform;
    AABB bounds;
    void *data;
} Shape;


// Box-shape
typedef struct
{
    float width;
    float height;    
} BoxShapeData;

typedef struct 
{
    float r;
} CircleShapeData;

typedef struct
{
    Vector2 *points;
    size_t count;
} PolygonShapeData;


void updateAABB(Shape *shape);
void computeBoxAABB(Shape *shape);
void computeCircleAABB(Shape *shape);
void computePolygonAABB(Shape *shape);

Shape createBox(float height, float width, Vector2 pos, float angle);
Shape *createPolygon(Vector2 points[], size_t count, Vector2 pos, float angle);
Shape createCircle(float r, Vector2 pos);
Shape *convertBoxToPoly(Shape *box);
void deleteShape(Shape *shape);


#endif