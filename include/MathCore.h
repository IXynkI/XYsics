#ifndef MATHCORE_H
#define MATHCORE_H

#include <math.h>
#include <stddef.h>
#include <Utils.h>
#include <structs/MathStructs.h>

#define EPSILON 1e-6f  // = 0.000001


typedef struct 
{
    Vector2 points[3];
    size_t size;
} Simplex;

//Basic
Vector2 getNormal(Vector2 *midPoint, Vector2 *edge, Vector2 *center);
Vector2 getPerpendicular(Vector2 *edge, Vector2 *towardOrigin);
Vector2 projectPointOnVector(Vector2 *point, Vector2 *vector);
Vector2 getCenter(Vector2 points[], size_t size);
Vector2 getClosestPointOnSegment(Vector2 *A, Vector2 *B, Vector2 *P, float *outDistanceSqr);
int comparePoints(const void *a, const void *b, void *arg);
void sortPolygonCenter(Vector2 polygon[], size_t size);
void simplexPush(Simplex *s, Vector2 *a);
void simplexDelLast(Simplex *s);
void simplexDel(Simplex *s, Vector2 *a);
float safeInv(float x);

//Clips the segment v1-v2 based on vector n and offset of o
//Result may have 2/1/0 points
ClippedPoints clip(Vector2 *v1, Vector2 *v2, Vector2 *n, float o);

//GJK
Vector2 getSupportPointPolygon(Vector2 verticies[], size_t count, Vector2 *d);
Vector2 getSupportPointSphere(Vector2 *center, double r, Vector2 *d);
void minkowskiSum (Vector2 verticies1[], size_t count1, Vector2 verticies2[], size_t count2, Vector2 result_return[]);
void minkowskiDiff (Vector2 verticies1[], size_t count1, Vector2 verticies2[], size_t count2, Vector2 result_return[]);
Vector2 getSupportPointMinkowskiDiff(Vector2 verticies1[], size_t l1, Vector2 verticies2[], size_t l2, Vector2 *d);
bool handleSimplex(Simplex *s, Vector2 *d);
bool gjkLineCase(Simplex *s, Vector2 *d);   
bool gjkTriangleCase(Simplex *s, Vector2 *d);

//Essential
bool compareVectors(Vector2 *a, Vector2 *b);
float dotProduct(Vector2 *a, Vector2 *b);
Vector2 normalize(Vector2 *a);
float getLength(Vector2 *a);
float crossProduct(Vector2 *a, Vector2 *b);
Vector2 crossProductVF(Vector2 *a, float f);
Vector2 addVectors(Vector2 *a, Vector2 *b);
Vector2 substractVectors(Vector2 *a, Vector2 *b);
Vector2 multiplyVectors(Vector2 *a, Vector2 *b);
Vector2 multiplyVectorF(Vector2 *a, float f);
Vector2 reverseVector(Vector2 *a);
void transpose(Matrix2 *m);
void rotateByMatrix(Vector2 *v, Matrix2 *m);
void rotateByAngle(Vector2 *v, float angle);
Vector2 rotateByMatrixReturn(Vector2 *v, Matrix2 *m);
Vector2 rotateByAngleReturn(Vector2 *v, float angle);

#endif