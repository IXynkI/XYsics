#ifndef GLOBALS_H
#define GLOBALS_H
#include <World.h>
#include <render/Camera.h>

#define CAMERA_MOVE_SPEED 1.0f
#define MAX_IMPULSE 10000.0f

extern float GRAVITY;

extern float windowW;
extern float windowH;
extern Camera camera;
extern World world;


#endif