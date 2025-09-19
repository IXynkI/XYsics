#ifndef CAMERA_H
#define CAMERA_H
#include <structs/MathStructs.h>

typedef struct Camera{
    Vector2 pos;
    float zoom;
} Camera;

//Will also include anchoring to the center of the screen
void moveCamera (Camera *camera, Vector2 newPos);

void resetCamera (Camera *camera);
void zoomCamera (Camera *camera, float newZoom);
void moveAndZoomCamera (Camera *camera, Vector2 newPos, float newZoom);

#endif