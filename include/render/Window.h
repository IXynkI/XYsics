#ifndef WINDOW_H
#define WINDOW_H

#include <X11/Xlib.h>
#include <stdbool.h>

extern Display *display;
extern Window window;
extern GC gc;
extern bool running;

bool windowInit(int width, int height, const char *title);
void windowRun(void (*drawCallback)(Display *, Window, GC, float));
void windowClose();

#endif
