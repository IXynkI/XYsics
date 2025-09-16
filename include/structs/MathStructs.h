#ifndef MATHSTRUCTS_H
#define MATHSTRUCTS_H
#include <stddef.h>


// Two dimensional vector.
typedef struct Vector2
{
    float x;
    float y;
} Vector2;

typedef struct AABB
{
    Vector2 min;
    Vector2 max;
} AABB;

typedef struct Matrix2
{
    float m00, m01;
    float m10, m11;
} Matrix2;


typedef struct Transform
{   
    //Rotation
    Matrix2 R;
    //Position  
    Vector2 pos;
} Transform;


typedef struct ClippedPoints
{
    Vector2 points[2];
    size_t count;
} ClippedPoints;

#endif