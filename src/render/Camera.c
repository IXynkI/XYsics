#include <render/Camera.h>

void moveCamera (Camera *camera, Vector2 newPos){
    camera->pos = newPos;
}
void resetCamera (Camera *camera){
    camera->pos = (Vector2){0, 0};
    camera->zoom = 1.0f;
}
void zoomCamera (Camera *camera, float newZoom){
    camera->zoom = newZoom;
}
void moveAndZoomCamera (Camera *camera, Vector2 newPos, float newZoom){
    moveCamera(camera, newPos);
    zoomCamera(camera, newZoom);
}