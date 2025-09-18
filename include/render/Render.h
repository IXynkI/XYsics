#ifndef RENDER_H
#define RENDER_H

#include <Shapes.h>
#include <render/Window.h>
#include <functions/Transform.h>
#include <Globals.h>

// Sets colors for background, foreground, lines and filling
void setGCStyle(unsigned int background, unsigned int foreground, unsigned int line, unsigned int fill);

void tick(Display *display, Window window, GC gc, float deltaTime);

void drawWholeScene();

void drawShape(Shape *shape);
void drawAABB(Shape *shape);
void drawPoint(Vector2 *p);
void drawLine(Vector2 *p1, Vector2 *p2);

void drawBox(Shape *shape);
void drawPolygon(Shape *shape);
void drawCircle(Shape *shape);

void beginRender();
void stopRender();

#endif