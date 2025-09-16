#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <structs/MathStructs.h>
#include <math.h>
#include <stddef.h>

//Transform
Matrix2 createRotationMatrix(float angle);
Matrix2 transposeMatrix(Matrix2 R);
Vector2 transformPoint(Transform t, Vector2 p);
void transformPoints(Transform t, Vector2 points[], size_t count, Vector2 result[]);
float getRotationalAngleRad(Matrix2 R);
float getRotationalAngleDeg(Matrix2 R);


#endif
