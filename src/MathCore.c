#include <MathCore.h>
#include <Utils.h>

// Basic

/*
midPoint = Middle point of an edge Vector
edge = Vector of a side of a convex polygon
center = Center of a convex polygon
*/
Vector2 getNormal(Vector2 *midPoint, Vector2 *edge, Vector2 *center)
{
    // Vector candidates for vector normal to side of a convex polygon
    Vector2 candidates[2] = {
        {-edge->y, edge->x},
        {edge->y, -edge->x}};

    Vector2 fromCenterToMid = (Vector2){midPoint->x - center->x, midPoint->y - center->y};

    for (int i = 0; i < 2; i++)
    {
        Vector2 *candidate = &candidates[i];

        // Checking if dot product of two vectors is equal or greater than 0
        if (dotProduct(candidate, &fromCenterToMid) >= 0)
        {
            return (Vector2){candidate->x, candidate->y};
        }
    }
    return (Vector2){0, 0};
}

// Getting perpendicluar from edge towards origin
Vector2 getPerpendicular(Vector2 *edge, Vector2 *towardOrigin)
{
    Vector2 result = {edge->x, -edge->y};

    if (dotProduct(&result, towardOrigin) < 0)
    {
        result.x = -result.x;
        result.y = -result.y;
    }
    return result;
}

Vector2 projectPointOnVector(Vector2 *point, Vector2 *vector)
{
    // Vector squared
    float vSq = dotProduct(vector, vector);
    if (vSq == 0)
    {
        return (Vector2){0, 0};
    }
    float scalar = dotProduct(point, vector) / vSq;
    return (Vector2){(vector->x * scalar), (vector->y * scalar)};
}

Vector2 getCenter(Vector2 points[], size_t size)
{
    Vector2 center = {0, 0};
    for (size_t i = 0; i < size; i++)
    {
        center.x += points[i].x;
        center.y += points[i].y;
    }

    center.x /= size;
    center.y /= size;

    return center;
}

Vector2 getClosestPointOnSegment(Vector2 *A, Vector2 *B, Vector2 *P, float *outDistanceSqr)
{
    Vector2 AB = substractVectors(B, A);
    Vector2 AP = substractVectors(P, A);

    float proj = dotProduct(&AP, &AB);
    float ABLenSQR = dotProduct(&AB, &AB);

    if (outDistanceSqr != NULL)
    {
        *outDistanceSqr = ABLenSQR;
    }

    if (ABLenSQR == 0)
        return *A;

    float d = proj / ABLenSQR;

    if (d <= 0)
        return *A;
    if (d >= 1)
        return *B;

    Vector2 closestPoint = {AB.x * d + A->x, AB.y * d + A->y};
    return closestPoint;
}

// Compares points based on angle relative to the center
int comparePoints(const void *a, const void *b, void *arg)
{
    const Vector2 *pointA = (const Vector2 *)a;
    const Vector2 *pointB = (const Vector2 *)b;
    Vector2 *center = (Vector2 *)arg;

    double angleA = atan2(pointA->y - center->y, pointA->x - center->x);
    double angleB = atan2(pointB->y - center->y, pointB->x - center->x);

    if (angleA < angleB)
        return -1;
    else if (angleA > angleB)
        return 1;
    else
        return 0;
}

// Sorts convex polygon's based on the angle relative to the center
void sortPolygonCenter(Vector2 polygon[], size_t size)
{
    Vector2 center = getCenter(polygon, size);

    qsort_r(polygon, size, sizeof(Vector2), comparePoints, &center);
}

void simplexPush(Simplex *s, Vector2 *a)
{
    s->points[2] = s->points[1];
    s->points[1] = s->points[0];
    s->points[0] = *a;
    if (s->size < 4)
    {
        s->size = s->size++;
    }
}

void simplexDelLast(Simplex *s)
{
    if (s->size <= 0)
    {
        return;
    }
    s->points[s->size - 1] = (Vector2){0, 0};
    s->size = s->size--;
}

void simplexDel(Simplex *s, Vector2 *a)
{
    if (s->size <= 0)
    {
        return;
    }
    size_t deletedIndex = -99;
    // Deleting and finding deletedIndex
    for (size_t i = 0; i < s->size; i++)
    {
        Vector2 iV = s->points[i];
        if ((iV.x - a->x < EPSILON) && (iV.y - a->y < EPSILON))
        {
            deletedIndex = i;
            break;
        }
    }
    // Checking if deleted obj was in the end of arr
    if (s->size - deletedIndex == 1)
    {
        s->size = s->size - 1;
        return;
    }

    // Moving all other object up in the arr
    for (size_t i = deletedIndex; i < s->size; i++)
    {
        if (i + 1 < 3)
        {
            s->points[i] = s->points[i + 1];
        }
    }
    s->size = s->size--;
}

float safeInv(float x){
    return ((x == INFINITY || x == 0.0f) ? 0.0f : 1.0f/x);
}


ClippedPoints clip(Vector2 *v1, Vector2 *v2, Vector2 *n, float o)
{
    ClippedPoints clipped;
    clipped.count = 0;
    float d1 = dotProduct(n, v1) - o;
    float d2 = dotProduct(n, v2) - o;

    if (d1 >= 0)
        clipped.points[clipped.count++] = *v1;
    if (d2 >= 0)
        clipped.points[clipped.count++] = *v2;

    if (d1 * d2 < 0)
    {
        Vector2 e = substractVectors(v2, v1);
        float u = d1 / (d1 - d2);
        e = multiplyVectorF(&e, u);
        e = addVectors(&e, v1);
        clipped.points[clipped.count++];
    }

    return clipped;
}

///// GJK

Vector2 getSupportPointPolygon(Vector2 verticies[], size_t count, Vector2 *d)
{
    float highest = -__FLT_MAX__;
    Vector2 support = verticies[0];

    for (size_t i = 0; i < count; i++)
    {
        float dot = dotProduct(&verticies[i], d);
        if (highest < dot)
        {
            highest = dot;
            support = verticies[i];
        }
    }
    return support;
}

Vector2 getSupportPointSphere(Vector2 *center, double r, Vector2 *d)
{
    Vector2 normalizied = normalize(d);
    return (Vector2){center->x + normalizied.x * r, center->y + normalizied.y * r};
}

void minkowskiSum(Vector2 verticies1[], size_t count1, Vector2 verticies2[], size_t count2, Vector2 result_return[])
{
    if (result_return == NULL)
        return;
    for (size_t i = 0; i < count1; i++)
    {
        Vector2 v1 = verticies1[i];
        for (size_t j = 0; j < count2; j++)
        {
            Vector2 v2 = verticies2[j];
            result_return[(count2 * i) + j] = (Vector2){v1.x + v2.x, v1.y + v2.y};
        }
    }
}
void minkowskiDiff(Vector2 verticies1[], size_t count1, Vector2 verticies2[], size_t count2, Vector2 result_return[])
{
    if (result_return == NULL)
        return;
    for (size_t i = 0; i < count1; i++)
    {
        Vector2 v1 = verticies1[i];
        for (size_t j = 0; j < count2; j++)
        {
            Vector2 v2 = verticies2[j];
            result_return[(count2 * i) + j] = (Vector2){v1.x - v2.x, v1.y - v2.y};
        }
    }
}

// Gets only hull point of minkowski difference
//  "d" - vector towards origin, if possible.
Vector2 getSupportPointMinkowskiDiff(Vector2 verticies1[], size_t l1, Vector2 verticies2[], size_t l2, Vector2 *d)
{
    // Support points from opposite vectors
    Vector2 support1 = getSupportPointPolygon(verticies1, l1, d);
    Vector2 negativeD = reverseVector(d);
    Vector2 support2 = getSupportPointPolygon(verticies2, l2, &negativeD);

    // Support point on Minkowski Diferrence
    return substractVectors(&support1, &support2);
}

bool handleSimplex(Simplex *s, Vector2 *d)
{
    if (s->size == 2)
    {
        return gjkLineCase(s, d);
    }
    return gjkTriangleCase(s, d);
}

bool gjkLineCase(Simplex *s, Vector2 *d)
{
    Vector2 A = s->points[0];
    Vector2 B = s->points[1];
    Vector2 AB = substractVectors(&B, &A);
    Vector2 AO = reverseVector(&A);

    // Checking if origin lies on the edge
    float cross = crossProduct(&AB, &AO);
    float dotABAO = dotProduct(&AB, &AO);
    float dotABAB = dotProduct(&AB, &AB);
    if (fabsf(cross) < EPSILON && dotABAO >= 0 && dotABAO <= dotABAB)
    {
        return true;
    }

    Vector2 perp = getPerpendicular(&AB, &AO);
    *d = perp;

    return false;
}
bool gjkTriangleCase(Simplex *s, Vector2 *d)
{
    Vector2 A = s->points[0];
    Vector2 B = s->points[1];
    Vector2 C = s->points[2];
    Vector2 AB = substractVectors(&B, &A);
    Vector2 AC = substractVectors(&C, &A);
    Vector2 AO = reverseVector(&A);

    Vector2 ABperp = getPerpendicular(&AB, &AO);
    Vector2 ACperp = getPerpendicular(&AC, &AO);

    // Checking if origin lies on one of the two new edges
    float cross = crossProduct(&AB, &AO);
    float dotABAO = dotProduct(&AB, &AO);
    float dotABAB = dotProduct(&AB, &AB);
    if (fabsf(cross) < EPSILON && dotABAO >= 0 && dotABAO <= dotABAB)
    {
        return true;
    }

    cross = crossProduct(&AC, &AO);
    float dotACAO = dotProduct(&AC, &AO);
    float dotACAC = dotProduct(&AC, &AC);
    if (fabsf(cross) < EPSILON && dotACAO >= 0 && dotACAO <= dotACAC)
    {
        return true;
    }

    // Checking if origin is in AB region
    if (dotProduct(&ABperp, &AO) > 0)
    {
        simplexDelLast(s);
        *d = ABperp;
        return false;
    }
    // Checking if origin is in AC region
    else if (dotProduct(&ACperp, &AO) > 0)
    {
        simplexDel(s, &B);
        *d = ACperp;
        return false;
    }
    return true;
}

////// Essential

bool compareVectors(Vector2 *a, Vector2 *b)
{
    return (fabsf(a->x - b->x) < EPSILON) && (fabsf(a->y - b->y) < EPSILON);
}

float dotProduct(Vector2 *a, Vector2 *b)
{
    return (a->x * b->x + a->y * b->y);
}

Vector2 normalize(Vector2 *a)
{
    float l = getLength(a);
    return (Vector2){a->x / l, a->y / l};
}

float getLength(Vector2 *a)
{
    return sqrtf(a->x * a->x + a->y * a->y);
}

Vector2 addVectors(Vector2 *a, Vector2 *b)
{
    return (Vector2){a->x + b->x, a->y + b->y};
}

Vector2 substractVectors(Vector2 *a, Vector2 *b)
{
    return (Vector2){a->x - b->x, a->y - b->y};
}

Vector2 multiplyVectors(Vector2 *a, Vector2 *b)
{
    return (Vector2){a->x * b->x, a->y * b->y};
}
Vector2 multiplyVectorF(Vector2 *a, float f)
{
    return (Vector2){a->x * f, a->y * f};
}

Vector2 reverseVector(Vector2 *a)
{
    return (Vector2){-a->x, -a->y};
}

float crossProduct(Vector2 *a, Vector2 *b)
{
    return a->x * b->y - a->y * b->x;
}

Vector2 crossProductVF(Vector2 *a, float f){
    return (Vector2){-f * a->y, f * a->x};
}


void transpose(Matrix2 *m)
{
    float temp = m->m01;
    m->m01 = m->m10;
    m->m10 = temp;
}

void rotateByMatrix(Vector2 *v, Matrix2 *m)
{
    v->x = m->m00 * v->x + m->m01 * v->y;
    v->y = m->m10 * v->x + m->m11 * v->y;
}
void rotateByAngle(Vector2 *v, float angle)
{
    float c = cosf(angle), s = sinf(angle);
    v->x = v->x * c - v->y * s;
    v->y = v->x * s + v->y * c;
}
Vector2 rotateByMatrixReturn(Vector2 *v, Matrix2 *m)
{
    Vector2 result;
    result.x = m->m00 * v->x + m->m01 * v->y;
    result.y = m->m10 * v->x + m->m11 * v->y;
    return result;
}
Vector2 rotateByAngleReturn(Vector2 *v, float angle)
{
    Vector2 result;
    float c = cosf(angle), s = sinf(angle);
    result.x = v->x * c - v->y * s;
    result.y = v->x * s + v->y * c;
    return result;
}
