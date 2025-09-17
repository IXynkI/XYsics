#include <Collision.h>
#include <Utils.h>
#include <MathCore.h>
#include <Globals.h>
#include <functions/Transform.h>

bool TestAABBOverlap(AABB *a, AABB *b)
{
    float d1x = a->min.x - b->max.x;
    float d1y = a->min.y - b->max.y;
    float d2x = b->min.x - a->max.x;
    float d2y = b->min.y - a->max.y;

    if (d1x > 0.0f || d1y > 0.0f)
        return false;
    if (d2x > 0.0f || d2y > 0.0f)
        return false;

    return true;
}

/*
    size1 - size of a first array
    size2 - size of a second array
    Arrays should be sorted based on angle relative to the center. Use sortPolygonCenter()
*/
bool TestPolygonOverlapSAT(Vector2 polygon1[], size_t size1, Vector2 polygon2[], size_t size2, CollisionPair *collPair)
{
    float minOverlap = INFINITY;
    Vector2 smallestAxis = {0, 0};

    for (size_t shape = 0; shape < 2; shape++)
    {
        Vector2 *poly = (shape == 0) ? polygon1 : polygon2;
        size_t size = (shape == 0) ? size1 : size2;

        for (size_t i = 0; i < size; i++)
        {
            // Ребро
            Vector2 a = poly[i];
            Vector2 b = poly[(i + 1) % size];
            Vector2 edge = {b.x - a.x, b.y - a.y};

            // Ось проекции
            Vector2 axis = {edge.y, -edge.x};
            axis = normalize(&axis);

            // Проекции полигона 1
            float min1 = INFINITY, max1 = -INFINITY;
            for (size_t j = 0; j < size1; j++)
            {
                float proj = dotProduct(&polygon1[j], &axis);
                if (proj < min1)
                    min1 = proj;
                if (proj > max1)
                    max1 = proj;
            }

            // Проекции полигона 2
            float min2 = INFINITY, max2 = -INFINITY;
            for (size_t j = 0; j < size2; j++)
            {
                float proj = dotProduct(&polygon2[j], &axis);
                if (proj < min2)
                    min2 = proj;
                if (proj > max2)
                    max2 = proj;
            }

            // Если есть зазор - нет пересечения
            if (max1 < min2 || max2 < min1)
                return false;
            // Вычисляем overlap (перекрытие)
            float overlap = fminf(max1, max2) - fmaxf(min1, min2);

            if (overlap < minOverlap)
            {
                minOverlap = overlap;
                smallestAxis = axis;
            }
        }
    }
    // Определяем направление MTV (от полигона1 к полигону2)
    Vector2 center1 = {0, 0}, center2 = {0, 0};
    for (size_t i = 0; i < size1; i++)
    {
        center1.x += polygon1[i].x;
        center1.y += polygon1[i].y;
    }
    center1.x /= (float)size1;
    center1.y /= (float)size1;

    for (size_t i = 0; i < size2; i++)
    {
        center2.x += polygon2[i].x;
        center2.y += polygon2[i].y;
    }
    center2.x /= (float)size2;
    center2.y /= (float)size2;

    Vector2 dir = {center2.x - center1.x, center2.y - center1.y};
    if (dotProduct(&dir, &smallestAxis) < 0)
    {
        smallestAxis.x = -smallestAxis.x;
        smallestAxis.y = -smallestAxis.y;
    }

    collPair->axis.x = smallestAxis.x * minOverlap;
    collPair->axis.y = smallestAxis.y * minOverlap;
    collPair->MTVlength = getLength(&collPair->axis);
    collPair->collided = true;

    return true;
}

// TODO: Caching normals inside data of a shape, since they cannot change their form
bool checkNarrowBoxCircle(Shape *box, Shape *circle, CollisionPair *collPair)
{
    BoxShapeData *boxData = (BoxShapeData *)box->data;
    CircleShapeData *circleData = (CircleShapeData *)circle->data;

    Vector2 boxToCircle = substractVectors(&circle->transform.pos, &box->transform.pos);

    // Converting circle to the local coordinates of box
    Matrix2 boxMatrix = box->transform.R;
    Vector2 circleInLocal = {
        boxToCircle.x * boxMatrix.m00 + boxToCircle.y * boxMatrix.m10,
        boxToCircle.x * boxMatrix.m01 + boxToCircle.y * boxMatrix.m11};

    float halfWidth = boxData->width * 0.5f;
    float halfHeight = boxData->height * 0.5f;

    Vector2 closestPoint = circleInLocal;
    if (closestPoint.x < -halfWidth)
        closestPoint.x = -halfWidth;
    else if (closestPoint.x > halfWidth)
        closestPoint.x = halfWidth;
    if (closestPoint.y < -halfHeight)
        closestPoint.y = -halfHeight;
    else if (closestPoint.y > halfHeight)
        closestPoint.y = halfHeight;

    Vector2 diff = substractVectors(&circleInLocal, &closestPoint);
    float dist = getLength(&diff);

    if (dist < circleData->r)
    {
        Vector2 localMTV;
        float penetration;
        Vector2 contacts[2];
        int contactCount = 0;

        if (dist > 1e-6f) // Circle outside
        {
            Vector2 normal = normalize(&diff);
            penetration = circleData->r - dist;
            localMTV = (Vector2){normal.x * penetration, normal.y * penetration};

            contacts[contactCount++] = closestPoint;

            // If in the corner, adding the second contact point from neighbouring edge
            if ((fabsf(closestPoint.x) == halfWidth) && (fabsf(closestPoint.y) == halfHeight))
            {
                contacts[contactCount++] = (Vector2){closestPoint.x, 0};
            }
        }
        else // Circle inside
        {
            float dx = halfWidth - fabsf(circleInLocal.x);
            float dy = halfHeight - fabsf(circleInLocal.y);

            if (dx < dy)
            {
                float sign = (circleInLocal.x > 0) ? 1.0f : -1.0f;
                localMTV = (Vector2){sign * dx, 0};
                penetration = dx;

                contacts[contactCount++] = (Vector2){sign * halfWidth, circleInLocal.y};
                // If in the corner, adding the second contact point from neighbouring edge
                if (fabsf(circleInLocal.y) >= halfHeight - 1e-6f)
                    contacts[contactCount++] = (Vector2){sign * halfWidth, (circleInLocal.y > 0 ? halfHeight : -halfHeight)};
            }
            else
            {
                float sign = (circleInLocal.y > 0) ? 1.0f : -1.0f;
                localMTV = (Vector2){0, sign * dy};
                penetration = dy;

                contacts[contactCount++] = (Vector2){circleInLocal.x, sign * halfHeight};
                // If in the corner, adding the second contact point from neighbouring edge
                if (fabsf(circleInLocal.x) >= halfWidth - 1e-6f)
                    contacts[contactCount++] = (Vector2){(circleInLocal.x > 0 ? halfWidth : -halfWidth), sign * halfHeight};
            }
        }

        // MTV to the world coordinates
        collPair->axis = localMTV;
        rotateByMatrix(&collPair->axis, &boxMatrix);
        collPair->MTVlength = getLength(&collPair->axis);

        // Contact points to the world coordinates
        Vector2 normal = normalize(&localMTV);
        for (int i = 0; i < contactCount; i++)
        {
            Vector2 worldContact = rotateByMatrixReturn(&contacts[i], &boxMatrix);
            worldContact = addVectors(&worldContact, &box->transform.pos);
            collPair->contactPoints[i] = worldContact;
            collPair->contactDepth[i] = dotProduct(&normal, &localMTV);
        }

        collPair->contactCount = contactCount;
        collPair->collided = true;

        return true;
    }

    collPair->collided = false;
    collPair->contactCount = 0;
    return false;
}
bool checkNarrowBoxBox(Shape *box0, Shape *box1, CollisionPair *collPair)
{
    BoxShapeData *boxData0 = (BoxShapeData *)box0->data;
    BoxShapeData *boxData1 = (BoxShapeData *)box1->data;

    size_t BOX_POINTS_COUNT = 4;

    // Building points array for actual SAT function
    Vector2 points0[BOX_POINTS_COUNT];
    Vector2 points1[BOX_POINTS_COUNT];

    points0[0] = (Vector2){boxData0->width / 2, boxData0->height / 2};
    points0[1] = (Vector2){-boxData0->width / 2, boxData0->height / 2};
    points0[2] = (Vector2){-boxData0->width / 2, -boxData0->height / 2};
    points0[3] = (Vector2){boxData0->width / 2, -boxData0->height / 2};

    points1[0] = (Vector2){boxData1->width / 2, boxData1->height / 2};
    points1[1] = (Vector2){-boxData1->width / 2, boxData1->height / 2};
    points1[2] = (Vector2){-boxData1->width / 2, -boxData1->height / 2};
    points1[3] = (Vector2){boxData1->width / 2, -boxData1->height / 2};

    transformPoints(box0->transform, points0, BOX_POINTS_COUNT, points0);
    transformPoints(box1->transform, points1, BOX_POINTS_COUNT, points1);

    // Normals for both boxes. Because we're checking boxes, it means that
    // only two out of normals of each box will be unique
    Vector2 normals[4];
    for (size_t i = 0; i < 2; i++)
    {
        // Getting normals for first box
        Vector2 currEdge = substractVectors(&points0[i + 1], &points0[i]);
        Vector2 perpen = getPerpendicular(&currEdge, &box0->transform.pos);
        normals[i] = reverseVector(&perpen);
        normals[i] = normalize(&normals[i]);
        // Getting normals for second box
        currEdge = substractVectors(&points1[i + 1], &points1[i]);
        perpen = getPerpendicular(&currEdge, &box1->transform.pos);
        normals[i + 2] = reverseVector(&perpen);
        normals[i + 2] = normalize(&normals[i + 2]);
    }

    // Only 4 overlaps possible for box shape
    float overlaps[4];
    // Iterating over normals and getting projections of vertecies on normal
    for (size_t i = 0; i < 4; i++)
    {
        float max0 = -INFINITY;
        float min0 = INFINITY;
        float max1 = -INFINITY;
        float min1 = INFINITY;
        Vector2 currNormal = normals[i];

        // Box0
        for (size_t j = 0; j < BOX_POINTS_COUNT; j++)
        {
            float proj = dotProduct(&points0[j], &currNormal);
            if (max0 < proj)
                max0 = proj;
            if (min0 > proj)
                min0 = proj;
        }
        // Box1
        for (size_t j = 0; j < BOX_POINTS_COUNT; j++)
        {
            float proj = dotProduct(&points1[j], &currNormal);
            if (max1 < proj)
                max1 = proj;
            if (min1 > proj)
                min1 = proj;
        }
        // There's no collision
        if (max0 < min1 || min0 > max1)
        {
            collPair->collided = false;
            return false;
        }
        float overlap = fmin(max0, max1) - fmax(min0, min1);

        overlaps[i] = overlap;
    }
    // Searching for minimal overlap
    float minOverlap = overlaps[0];
    size_t minOverlapIndex = 0;
    for (size_t i = 1; i < 4; i++)
    {
        if (overlaps[i] < minOverlap)
        {
            minOverlap = overlaps[i];
            minOverlapIndex = i;
        }
    }

    collPair->MTVlength = minOverlap;
    collPair->collided = true;

    // Checking if normal is pointing in right direction through dot product and vector from box0 to box1
    Vector2 Box0Box1 = substractVectors(&box1->transform.pos, &box0->transform.pos);
    if (dotProduct(&Box0Box1, &normals[minOverlapIndex]) < 0)
    {
        collPair->axis = reverseVector(&normals[minOverlapIndex]);
    }
    else
    {
        collPair->axis = normals[minOverlapIndex];
    }

    // Searching for reference edge of box0
    Vector2 referenceEdge = substractVectors(&points0[1], &points0[0]);
    Vector2 referenceA = points0[0];
    Vector2 referenceB = points0[1];

    for (size_t i = 1; i < BOX_POINTS_COUNT; i++)
    {
        Vector2 currEdge = substractVectors(&points0[(i + 1) % BOX_POINTS_COUNT], &points0[i]);
        if (fabsf(dotProduct(&currEdge, &collPair->axis)) < fabsf(dotProduct(&referenceEdge, &collPair->axis)))
        {
            referenceEdge = currEdge;
            referenceA = points0[i];
            referenceB = points0[(i + 1) % BOX_POINTS_COUNT];
        }
    }
    Vector2 referenceNormal = getPerpendicular(&referenceEdge, &box0->transform.pos);
    referenceNormal = reverseVector(&referenceNormal);
    referenceNormal = normalize(&referenceNormal);
    Vector2 referenceTangent = normalize(&referenceEdge);

    // Searching for incident edge of box1
    Vector2 incidentEdge = substractVectors(&points1[1], &points1[0]);
    Vector2 incidentPerpen = getPerpendicular(&incidentEdge, &box1->transform.pos);
    incidentPerpen = normalize(&incidentPerpen);
    Vector2 incidentA = points1[0];
    Vector2 incidentB = points1[1];
    for (size_t i = 1; i < BOX_POINTS_COUNT; i++)
    {
        Vector2 currEdge = substractVectors(&points1[(i + 1) % BOX_POINTS_COUNT], &points1[i]);
        Vector2 currPerpen = getPerpendicular(&currEdge, &box1->transform.pos);
        currPerpen = normalize(&currPerpen);
        if (fabsf(dotProduct(&currPerpen, &referenceNormal)) > fabsf(dotProduct(&incidentPerpen, &referenceNormal)))
        {
            incidentEdge = currEdge;
            incidentPerpen = currPerpen;
            incidentA = points1[i];
            incidentB = points1[(i + 1) % BOX_POINTS_COUNT];
        }
    }

    Vector2 clippingPlane1 = reverseVector(&referenceTangent);
    Vector2 clippingPlane2 = referenceTangent;

    float clippingPlane1Offset = dotProduct(&clippingPlane1, &referenceA);
    float clippingPlane2Offset = dotProduct(&clippingPlane2, &referenceB);
    ClippedPoints clippedPoints;
    clippedPoints.count = 0;
    clippedPoints.points[0] = (Vector2){0, 0};
    clippedPoints.points[1] = (Vector2){0, 0};
    clippedPoints = clip(&incidentA, &incidentB, &clippingPlane1, clippingPlane1Offset);

    if (clippedPoints.count < 2)
    {
        return;
    }
    clippedPoints = clip(&clippedPoints.points[0], &clippedPoints.points[1], &clippingPlane2, clippingPlane2Offset);

    if (clippedPoints.count < 2)
    {
        return;
    }

    // Checking which point of reference aligns most with reference normal
    float max = fmax(
        dotProduct(&referenceNormal, &referenceA),
        dotProduct(&referenceNormal, &referenceB));

    if (dotProduct(&referenceNormal, &clippedPoints.points[0]) - max < 0.0)
    {
        clippedPoints.points[0] = clippedPoints.points[1];
        clippedPoints.points[1] = (Vector2){0, 0};
        clippedPoints.count--;
    }
    if (dotProduct(&referenceNormal, &clippedPoints.points[1]) - max < 0.0)
    {
        clippedPoints.points[1] = (Vector2){0, 0};
        clippedPoints.count--;
    }

    for (size_t i = 0; i < clippedPoints.count; i++)
    {
        collPair->contactPoints[i] = clippedPoints.points[i];
    }
    collPair->contactCount = clippedPoints.count;

    float depth0 = max - dotProduct(&referenceNormal, &clippedPoints.points[0]);
    float depth1 = max - dotProduct(&referenceNormal, &clippedPoints.points[1]);
    collPair->contactDepth[0] = depth0;
    collPair->contactDepth[1] = depth1;
}
bool checkNarrowCircleCircle(Shape *circle0, Shape *circle1, CollisionPair *collPair)
{
    collPair->collided = false;
    collPair->contactCount = 0;

    CircleShapeData *data0 = (CircleShapeData *)circle0->data;
    CircleShapeData *data1 = (CircleShapeData *)circle1->data;

    Vector2 center0 = circle0->transform.pos;
    Vector2 center1 = circle1->transform.pos;

    Vector2 dist = substractVectors(&center0, &center1);

    float distSQR = (dist.x * dist.x) + (dist.y * dist.y);
    float sumOfR = data0->r + data1->r;

    if (distSQR <= sumOfR * sumOfR)
    {
        float distL = getLength(&dist);

        float penetration = sumOfR - distL;
        dist = normalize(&dist);

        float offset = data0->r - (penetration * 0.5f);
        Vector2 contactOffsetFromCenter = multiplyVectorF(&dist, offset);
        Vector2 contactPoint = addVectors(&center0, &contactOffsetFromCenter);
        collPair->contactPoints[collPair->contactCount++] = contactPoint;

        collPair->axis = (Vector2){dist.x * penetration, dist.y * penetration};
        collPair->MTVlength = penetration;
        collPair->contactDepth[collPair->contactCount] = penetration / 2;

        collPair->collided = true;
        return true;
    }
    return false;
}
bool checkNarrowPolygonCircle(Shape *polygon, Shape *circle, CollisionPair *collPair)
{

    if(polygon->type != SHAPE_POLYGON){
        printf("Polygon isn't polygon");
    }
    PolygonShapeData *polygonData = (PolygonShapeData *)polygon->data;

    Vector2 transformedPoints[polygonData->count];
    transformPoints(polygon->transform, polygonData->points, polygonData->count, transformedPoints);

    collPair->contactCount = 0;
    collPair->collided = false;
    CircleShapeData *circleData = (CircleShapeData *)circle->data;
    

    Vector2 closestPoint = transformedPoints[0];

    for (size_t i = 0; i < polygonData->count; i++)
    {
        Vector2 closestPointCandidate = {0, 0};
        // Searching for closest point on a polygon to the center of circle
        Vector2 edge = substractVectors(&transformedPoints[(i + 1) % polygonData->count], &transformedPoints[i]);
        Vector2 centerToVertex = substractVectors(&circle->transform.pos, &transformedPoints[i]);

        float dot = dotProduct(&edge, &centerToVertex);

        float edgeL = getLength(&edge);
        float edgeLSQR = edgeL * edgeL;

        // Coefficient of intersection between perpendicular from point to line segment
        float t = dot / edgeLSQR;

        // If t<0 or t>1 then there's no intersection. If t is between 0 and 1 then there's an intersection and therefore projection is on the line segment
        if (t >= 0.0f && t <= 1.0f)
        {
            // Offset based on coefficient.
            Vector2 offset = multiplyVectorF(&edge, t);

            Vector2 proj = addVectors(&transformedPoints[i], &offset);
            closestPointCandidate = proj;
        }
        else if (t > 1)
        {
            closestPointCandidate = transformedPoints[(i + 1) % polygonData->count];
        }
        else
        { // t < 0
            closestPointCandidate = transformedPoints[i];
        }

        // Center to Closest point and Closest point candidate
        Vector2 centerToCP = substractVectors(&closestPoint, &circle->transform.pos);
        Vector2 centerToCPC = substractVectors(&closestPointCandidate, &circle->transform.pos);

        if (dotProduct(&centerToCP, &centerToCP) >= dotProduct(&centerToCPC, &centerToCPC))
        {
            closestPoint = closestPointCandidate;
        }
    }
    Vector2 centerToCP = substractVectors(&closestPoint, &circle->transform.pos);
    float centerToCPLength = getLength(&centerToCP);
    // If distance to closest point > radius then there's no intersection
    if (centerToCPLength > circleData->r)
    {
        collPair->collided = false;
        return false;
    }

    Vector2 separationNormal = substractVectors(&circle->transform.pos, &closestPoint);
    separationNormal = normalize(&separationNormal);
    float penetration = circleData->r - centerToCPLength;
    collPair->collided = true;
    collPair->contactDepth[collPair->contactCount] = penetration;
    collPair->contactPoints[collPair->contactCount++] = closestPoint;
    collPair->axis = multiplyVectorF(&separationNormal, penetration);
    collPair->MTVlength = penetration;
    return true;
}
bool checkNarrowPolygonPolygon(Shape *polygon0, Shape *polygon1, CollisionPair *collPair)
{
    PolygonShapeData *polyData0 = (PolygonShapeData *)polygon0->data;
    PolygonShapeData *polyData1 = (PolygonShapeData *)polygon1->data;

    Vector2 transformedPoints0[polyData0->count];
    Vector2 transformedPoints1[polyData1->count];
    transformPoints(polygon0->transform, polyData0->points, polyData0->count, transformedPoints0);
    transformPoints(polygon1->transform, polyData1->points, polyData1->count, transformedPoints1);

    Vector2 separationNormal;
    float minOverlap = INFINITY;

    // First polygon
    for (size_t i = 0; i < polyData0->count; i++)
    {
        Vector2 edge = substractVectors(&transformedPoints0[(i + 1) % polyData0->count], &transformedPoints0[i]);
        // Normal will be pointing towards center
        Vector2 currNormal = getPerpendicular(&edge, &polygon0->transform.pos);
        // Reversing so its pointing outwards
        currNormal = reverseVector(&currNormal);
        currNormal = normalize(&currNormal);

        float max0 = -INFINITY;
        float min0 = INFINITY;
        float max1 = -INFINITY;
        float min1 = INFINITY;

        // Comparing normal with first polygon
        for (size_t j = 0; j < polyData0->count; j++)
        {
            float proj = dotProduct(&transformedPoints0[j], &currNormal);
            if (max0 < proj)
                max0 = proj;
            if (min0 > proj)
                min0 = proj;
        }
        // Comparing normal with second polygon
        for (size_t j = 0; j < polyData1->count; j++)
        {
            float proj = dotProduct(&transformedPoints1[j], &currNormal);
            if (max1 < proj)
                max1 = proj;
            if (min1 > proj)
                min1 = proj;
        }
        // There's no collision
        if (max0 < min1 || min0 > max1)
        {
            collPair->collided = false;
            return false;
        }
        float overlap = fmin(max0, max1) - fmax(min0, min1);
        if (minOverlap > overlap)
        {
            minOverlap = overlap;
            separationNormal = currNormal;
        }
    }
    // Second polygon
    for (size_t i = 0; i < polyData1->count; i++)
    {
        Vector2 edge = substractVectors(&transformedPoints1[(i + 1) % polyData1->count], &transformedPoints1[i]);
        // Normal will be pointing towards center
        Vector2 currNormal = getPerpendicular(&edge, &polygon1->transform.pos);
        // Reversing so its pointing outwards
        currNormal = reverseVector(&currNormal);
        currNormal = normalize(&currNormal);

        float max0 = -INFINITY;
        float min0 = INFINITY;
        float max1 = -INFINITY;
        float min1 = INFINITY;

        // Comparing normal with first polygon
        for (size_t j = 0; j < polyData0->count; j++)
        {
            float proj = dotProduct(&currNormal, &transformedPoints0[j]);
            if (max0 < proj)
                max0 = proj;
            if (min0 > proj)
                min0 = proj;
        }
        // Comparing normal with second polygon
        for (size_t j = 0; j < polyData1->count; j++)
        {
            float proj = dotProduct(&transformedPoints1[j], &currNormal);
            if (max1 < proj)
                max1 = proj;
            if (min1 > proj)
                min1 = proj;
        }
        // There's no collision
        if (max0 < min1 || min0 > max1)
        {
            collPair->collided = false;
            return false;
        }
        float overlap = fmin(max0, max1) - fmax(min0, min1);
        if (minOverlap > overlap)
        {
            minOverlap = overlap;
            separationNormal = currNormal;
        }
    }
    collPair->collided = true;
    collPair->MTVlength = minOverlap;

    // Checking if normal is pointing in right direction through dot product and vector from box0 to box1
    Vector2 Polygon0Polygon1 = substractVectors(&polygon1->transform.pos, &polygon0->transform.pos);
    if (dotProduct(&Polygon0Polygon1, &separationNormal) < 0)
    {
        collPair->axis = reverseVector(&separationNormal);
    }
    else
    {
        collPair->axis = separationNormal;
    }

    // Searching for reference edge of polygon0
    Vector2 referenceEdge = substractVectors(&transformedPoints0[1], &transformedPoints0[0]);
    Vector2 referenceA = transformedPoints0[0];
    Vector2 referenceB = transformedPoints0[1];

    for (size_t i = 1; i < polyData0->count; i++)
    {
        Vector2 currEdge = substractVectors(&transformedPoints0[(i + 1) % polyData0->count], &transformedPoints0[i]);
        if (fabsf(dotProduct(&currEdge, &collPair->axis)) < fabsf(dotProduct(&referenceEdge, &collPair->axis)))
        {
            referenceEdge = currEdge;
            referenceA = transformedPoints0[i];
            referenceB = transformedPoints0[(i + 1) % polyData0->count];
        }
    }
    Vector2 referenceNormal = getPerpendicular(&referenceEdge, &polygon0->transform.pos);
    referenceNormal = reverseVector(&referenceNormal);
    referenceNormal = normalize(&referenceNormal);
    Vector2 referenceTangent = normalize(&referenceEdge);

    // Searching for incident edge of box1
    Vector2 incidentEdge = substractVectors(&transformedPoints1[1], &transformedPoints1[0]);
    Vector2 incidentPerpen = getPerpendicular(&incidentEdge, &polygon1->transform.pos);
    incidentPerpen = normalize(&incidentPerpen);
    Vector2 incidentA = transformedPoints1[0];
    Vector2 incidentB = transformedPoints1[1];
    for (size_t i = 1; i < polyData1->count; i++)
    {
        Vector2 currEdge = substractVectors(&transformedPoints1[(i + 1) % polyData1->count], &transformedPoints1[i]);
        Vector2 currPerpen = getPerpendicular(&currEdge, &polygon1->transform.pos);
        currPerpen = normalize(&currPerpen);
        if (fabsf(dotProduct(&currPerpen, &referenceNormal)) > fabsf(dotProduct(&incidentPerpen, &referenceNormal)))
        {
            incidentEdge = currEdge;
            incidentPerpen = currPerpen;
            incidentA = transformedPoints1[i];
            incidentB = transformedPoints1[(i + 1) % polyData1->count];
        }
    }

    Vector2 clippingPlane1 = reverseVector(&referenceTangent);
    Vector2 clippingPlane2 = referenceTangent;

    float clippingPlane1Offset = dotProduct(&clippingPlane1, &referenceA);
    float clippingPlane2Offset = dotProduct(&clippingPlane2, &referenceB);
    ClippedPoints clippedPoints;
    clippedPoints.count = 0;
    clippedPoints.points[0] = (Vector2){0, 0};
    clippedPoints.points[1] = (Vector2){0, 0};
    clippedPoints = clip(&incidentA, &incidentB, &clippingPlane1, clippingPlane1Offset);

    if (clippedPoints.count < 2)
    {
        return;
    }
    clippedPoints = clip(&clippedPoints.points[0], &clippedPoints.points[1], &clippingPlane2, clippingPlane2Offset);

    if (clippedPoints.count < 2)
    {
        return;
    }

    // Checking which point of reference aligns most with reference normal
    float max = fmax(
        dotProduct(&referenceNormal, &referenceA),
        dotProduct(&referenceNormal, &referenceB));

    if (dotProduct(&referenceNormal, &clippedPoints.points[0]) - max < 0.0)
    {
        clippedPoints.points[0] = clippedPoints.points[1];
        clippedPoints.points[1] = (Vector2){0, 0};
        clippedPoints.count--;
    }
    if (dotProduct(&referenceNormal, &clippedPoints.points[1]) - max < 0.0)
    {
        clippedPoints.points[1] = (Vector2){0, 0};
        clippedPoints.count--;
    }

    for (size_t i = 0; i < clippedPoints.count; i++)
    {
        collPair->contactPoints[i] = clippedPoints.points[i];
    }
    collPair->contactCount = clippedPoints.count;

    float depth0 = max - dotProduct(&referenceNormal, &clippedPoints.points[0]);
    float depth1 = max - dotProduct(&referenceNormal, &clippedPoints.points[1]);
    collPair->contactDepth[0] = depth0;
    collPair->contactDepth[1] = depth1;
    return true;
}
// Temporary function. Don't want to do it now
bool checkNarrowBoxPolygon(Shape *box, Shape *polygon, CollisionPair *collPair)
{
    // Making polygon out of box
    Shape *polygon0 = convertBoxToPoly(box);

    return checkNarrowPolygonPolygon(polygon0, polygon, collPair);
}

bool GJK(Vector2 verticies1[], size_t count1, Vector2 verticies2[], size_t count2)
{
    Vector2 ORIGIN = (Vector2){0, 0};
    // Vector form from the centers of shapes
    Vector2 center1 = getCenter(verticies1, count1);
    Vector2 center2 = getCenter(verticies2, count2);
    Vector2 sub = substractVectors(&center1, &center2);
    Vector2 d = normalize(&sub);

    Simplex simplex = {0};

    // Support point on Minkowski Diferrence
    Vector2 diffMinkSupport = getSupportPointMinkowskiDiff(verticies1, count1, verticies2, count2, &d);

    simplexPush(&simplex, &diffMinkSupport);

    // Vector from new point towards the origin. For now origin is = (0,0)
    d = substractVectors(&ORIGIN, &simplex.points[0]);

    while (true)
    {
        // Getting new support point
        diffMinkSupport = getSupportPointMinkowskiDiff(verticies1, count1, verticies2, count2, &d);

        // Checking if new support point is "passing" the origin
        if (dotProduct(&diffMinkSupport, &d) < 0)
        {
            return false;
        }
        simplexPush(&simplex, &diffMinkSupport);

        if (handleSimplex(&simplex, &d))
        {
            return true;
        }
    }
}

void initCollisionPairs(CollisionPairs *collPairs, size_t capacity)
{
    collPairs->pairs = malloc(sizeof(CollisionPair) * capacity);
    collPairs->count = 0;
    collPairs->capacity = capacity;
}
void addCollisionPair(CollisionPairs *collPairs, CollisionPair newCollisionPair)
{
    if (collPairs->count >= collPairs->capacity)
    {
        collPairs->capacity = collPairs->capacity > 0 ? collPairs->capacity * 2 : 16;
        CollisionPair *pairs = realloc(collPairs->pairs, sizeof(CollisionPair) * collPairs->capacity);
        if (!pairs)
        {
            fprintf(stderr, "Failed to realloc collision pairs\n");
            exit(1);
        }
        collPairs->pairs = pairs;
    }
    collPairs->pairs[collPairs->count++] = newCollisionPair;
}
void freeCollisionPairs(CollisionPairs *collPairs)
{
    free(collPairs->pairs);
    collPairs->count = 0;
    collPairs->capacity = 0;
    collPairs->pairs = NULL;
}

int compareAABB_X(const void *p1, const void *p2)
{
    const RigidBody *a = *(const RigidBody **)p1;
    const RigidBody *b = *(const RigidBody **)p2;
    if (a->shape.bounds.min.x < b->shape.bounds.min.x)
        return -1;
    if (a->shape.bounds.min.x > b->shape.bounds.min.x)
        return 1;
    return 0;
}

void sweepAndPrune(RigidBody **bodies, size_t count, CollisionPairs *candidates)
{
    qsort(bodies, count, sizeof(RigidBody *), compareAABB_X);
    for (size_t i = 0; i < count; i++)
    {
        RigidBody *a = bodies[i];
        for (size_t j = i + 1; j < count; j++)
        {
            RigidBody *b = bodies[j];

            if (b->shape.bounds.min.x > a->shape.bounds.max.x)
            {
                break;
            }

            if (a->shape.bounds.max.y < b->shape.bounds.min.y ||
                a->shape.bounds.min.y > b->shape.bounds.max.y)
            {
                continue;
            }

            CollisionPair potentialCollision = {i, j};
            addCollisionPair(candidates, potentialCollision);
        }
    }
}

void checkNarrow(RigidBody **bodies, CollisionPairs *collPairs, CollisionPairs *canditates)
{
    for (size_t i = 0; i < collPairs->count; i++)
    {
        Shape *shapeA = &bodies[collPairs->pairs[i].idA]->shape;
        Shape *shapeB = &bodies[collPairs->pairs[i].idB]->shape;

        ShapeType typeA = shapeA->type;
        ShapeType typeB = shapeB->type;

        // Messy solution. Might rework in future
        if (typeA == SHAPE_CIRCLE)
        {
            if (typeB == SHAPE_CIRCLE)
            {
                CollisionPair narrowResult;
                checkNarrowCircleCircle(shapeA, shapeB, &narrowResult);
                if (narrowResult.collided == true)
                {
                    narrowResult.idA = collPairs->pairs[i].idA;
                    narrowResult.idB = collPairs->pairs[i].idB;
                    addCollisionPair(canditates, narrowResult);
                }
            }
            if (typeB == SHAPE_BOX)
            {
                CollisionPair narrowResult;
                checkNarrowBoxCircle(shapeB, shapeA, &narrowResult);
                if (narrowResult.collided == true)
                {
                    narrowResult.idA = collPairs->pairs[i].idA;
                    narrowResult.idB = collPairs->pairs[i].idB;
                    addCollisionPair(canditates, narrowResult);
                }
            }
            if (typeB == SHAPE_POLYGON)
            {
                CollisionPair narrowResult;
                checkNarrowPolygonCircle(shapeB, shapeA, &narrowResult);
                if (narrowResult.collided == true)
                {
                    narrowResult.idA = collPairs->pairs[i].idA;
                    narrowResult.idB = collPairs->pairs[i].idB;
                    addCollisionPair(canditates, narrowResult);
                }
            }
        }
        else if (typeA == SHAPE_BOX)
        {
            if (typeB == SHAPE_CIRCLE)
            {
                CollisionPair narrowResult;
                checkNarrowBoxCircle(shapeA, shapeB, &narrowResult);
                if (narrowResult.collided == true)
                {
                    narrowResult.idA = collPairs->pairs[i].idA;
                    narrowResult.idB = collPairs->pairs[i].idB;
                    addCollisionPair(canditates, narrowResult);
                }
            }
            if (typeB == SHAPE_BOX)
            {
                CollisionPair narrowResult;
                checkNarrowBoxBox(shapeA, shapeB, &narrowResult);
                if (narrowResult.collided == true)
                {
                    narrowResult.idA = collPairs->pairs[i].idA;
                    narrowResult.idB = collPairs->pairs[i].idB;
                    addCollisionPair(canditates, narrowResult);
                }
            }
            if (typeB == SHAPE_POLYGON)
            {
                CollisionPair narrowResult;
                checkNarrowBoxPolygon(shapeA, shapeB, &narrowResult);
                if (narrowResult.collided == true)
                {
                    narrowResult.idA = collPairs->pairs[i].idA;
                    narrowResult.idB = collPairs->pairs[i].idB;
                    addCollisionPair(canditates, narrowResult);
                }
            }
        }
        else if (typeA == SHAPE_POLYGON)
        {
            if (typeB == SHAPE_CIRCLE)
            {
                CollisionPair narrowResult;
                checkNarrowPolygonCircle(shapeA, shapeB, &narrowResult);
                if (narrowResult.collided == true)
                {
                    narrowResult.idA = collPairs->pairs[i].idA;
                    narrowResult.idB = collPairs->pairs[i].idB;
                    addCollisionPair(canditates, narrowResult);
                }
            }
            if (typeB == SHAPE_BOX)
            {
                CollisionPair narrowResult;
                checkNarrowBoxPolygon(shapeB, shapeA, &narrowResult);
                if (narrowResult.collided == true)
                {
                    narrowResult.idA = collPairs->pairs[i].idA;
                    narrowResult.idB = collPairs->pairs[i].idB;
                    addCollisionPair(canditates, narrowResult);
                }
            }
            if (typeB == SHAPE_POLYGON)
            {
                CollisionPair narrowResult;
                checkNarrowPolygonPolygon(shapeA, shapeB, &narrowResult);
                if (narrowResult.collided == true)
                {
                    narrowResult.idA = collPairs->pairs[i].idA;
                    narrowResult.idB = collPairs->pairs[i].idB;
                    addCollisionPair(canditates, narrowResult);
                }
            }
        }
    }
}

void resolveCollisionPairs(RigidBody **bodies, CollisionPairs *collPairs)
{
    for (size_t i = 0; i < collPairs->count; i++)
    {
        resolveCollisionPair(bodies, &collPairs->pairs[i]);
    }
}
void resolveCollisionPair(RigidBody **bodies, CollisionPair *collPair)
{

    RigidBody *bodyA = bodies[collPair->idA];
    RigidBody *bodyB = bodies[collPair->idB];

    // Correcting position

    float invMassA = safeInv(bodyA->mass);
    float invMassB = safeInv(bodyB->mass);
    float sumInvMass = invMassA + invMassB;

    Vector2 correctionA = multiplyVectorF(&collPair->axis, collPair->MTVlength * (invMassA / sumInvMass));
    Vector2 correctionB = multiplyVectorF(&collPair->axis, collPair->MTVlength * (invMassB / sumInvMass));
    // If either body is immovable (mass = INFINITY), push only the movable one
    if (bodyA->mass == INFINITY)
    {
        bodyB->shape.transform.pos = addVectors(&bodyB->shape.transform.pos, &correctionB);
    }
    else if (bodyB->mass == INFINITY)
    {
        bodyA->shape.transform.pos = substractVectors(&bodyA->shape.transform.pos, &correctionA);
    }
    else
    {
        bodyA->shape.transform.pos = substractVectors(&bodyA->shape.transform.pos, &correctionA);
        bodyB->shape.transform.pos = addVectors(&bodyB->shape.transform.pos, &correctionB);
    }

    // Impulses for each contact point
    for (size_t i = 0; i < collPair->contactCount; i++)
    {
        // Relative vectors from centers to contact point or points
        Vector2 rA = substractVectors(&collPair->contactPoints[i], &bodyA->shape.transform.pos);
        Vector2 rB = substractVectors(&collPair->contactPoints[i], &bodyB->shape.transform.pos);
        Vector2 crossA = crossProductVF(&rA, bodyA->angularVel);
        Vector2 crossB = crossProductVF(&rB, bodyB->angularVel);

        // Velocity in contact point
        Vector2 velocityA = addVectors(&bodyA->linearVel, &crossA);
        Vector2 velocityB = addVectors(&bodyB->linearVel, &crossB);

        // Relative velocity in contact point
        Vector2 relativeVelocity = substractVectors(&velocityB, &velocityA);

        float velAlongNormal = dotProduct(&relativeVelocity, &collPair->axis);
        if (velAlongNormal > 0)
        {
            continue;
        }
        float denom = (1 / bodyA->mass + 1 / bodyB->mass) +
                      (powf(crossProduct(&rA, &collPair->axis), 2) / bodyA->momentOfInertia) +
                      (powf(crossProduct(&rB, &collPair->axis), 2) / bodyB->momentOfInertia);

        float bounciness = fminf(bodyA->restitution, bodyB->restitution);
        float j = -(1 + bounciness) * (dotProduct(&relativeVelocity, &collPair->axis)) / denom;

        Vector2 impulseOnContact = multiplyVectorF(&collPair->axis, j);

        // Linear
        if (bodyA->mass != INFINITY)
        {
            Vector2 linearChangeA = multiplyVectorF(&impulseOnContact, 1 / bodyA->mass);
            bodyA->linearVel = substractVectors(&bodyA->linearVel, &linearChangeA);
        }

        if (bodyB->mass != INFINITY)
        {
            Vector2 linearChangeB = multiplyVectorF(&impulseOnContact, 1 / bodyB->mass);
            bodyB->linearVel = addVectors(&bodyB->linearVel, &linearChangeB);
        }

        // Rotation
        if (bodyA->momentOfInertia != INFINITY)
            bodyA->angularVel -= crossProduct(&rA, &impulseOnContact) / bodyA->momentOfInertia;

        if (bodyB->momentOfInertia != INFINITY)
            bodyB->angularVel += crossProduct(&rB, &impulseOnContact) / bodyB->momentOfInertia;
    }
}

void checkAndResolveCollisions(World *w)
{
    if (w->bodies_count == 0 || w->bodies_count == 1)
    {
        printf("Collision check aborted");
        return;
    }
    // For filling in broad phase
    CollisionPairs collPairsBroad;
    initCollisionPairs(&collPairsBroad, w->capacity);

    // Will be filled based on result of Narrow phase
    CollisionPairs collPairsNarrow;
    initCollisionPairs(&collPairsNarrow, w->capacity);

    // Broad phase
    sweepAndPrune(w->bodies, w->bodies_count, &collPairsBroad);

    checkNarrow(w->bodies, &collPairsBroad, &collPairsNarrow);

    resolveCollisionPairs(w->bodies, &collPairsNarrow);
}
