#include <functions/Transform.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
///// Transform

Matrix2 createRotationMatrix(float angle)
{
    Matrix2 R;
    //Getting radians from degrees
    float angleRad = angle * (M_PI / 180.0f);
    float c = cosf(angleRad);
    float s = sinf(angleRad);
    R.m00 = c;
    R.m01 = -s;
    R.m10 = s;
    R.m11 = c;
    return R;
}

Matrix2 transposeMatrix(Matrix2 R)
{
    Matrix2 Rt;
    Rt.m00 = R.m00;
    Rt.m01 = R.m10;
    Rt.m10 = R.m01;
    Rt.m11 = R.m11;
    return Rt;
}

Vector2 transformPoint(Transform t, Vector2 p)
{
    Vector2 result;
    // Rotation
    result.x = (float)t.R.m00 * (float)p.x + (float)t.R.m01 * (float)p.y;
    result.y = (float)t.R.m10 * (float)p.x + (float)t.R.m11 * (float)p.y;
    // Position
    result.x += (float)t.pos.x;
    result.y += (float)t.pos.y;

    return result;
}

void transformPoints(Transform t, Vector2 points[], size_t count, Vector2 result[])
{
    for (size_t i = 0; i < count; i++)
    {
        result[i] = transformPoint(t, points[i]);
    }
}

float getRotationalAngleRad(Matrix2 R){
    return atan2(R.m10, R.m00);
}
float getRotationalAngleDeg(Matrix2 R){
    return getRotationalAngleRad(R) * (180 / M_PI);
}