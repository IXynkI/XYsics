#define _POSIX_C_SOURCE 199309L

// window/Window.c
#include <render/Window.h>
#include <Globals.h>

#include <X11/Xutil.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdio.h>

#include <time.h>

Display *display;
Window window;
GC gc;
bool running = true;

bool windowInit(int width, int height, const char *title)
{
    display = XOpenDisplay(NULL);
    if (!display)
    {
        fprintf(stderr, "Wasn't able to open up the display\n");
        return false;
    }

    int screen = DefaultScreen(display);
    window = XCreateSimpleWindow(display,
                                 RootWindow(display, screen),
                                 10, 10, width, height, 1,
                                 BlackPixel(display, screen),
                                 WhitePixel(display, screen));

    XSetStandardProperties(display, window, title, title, None, NULL, 0, NULL);
    XSelectInput(display, window, ExposureMask | StructureNotifyMask | ButtonPressMask | ButtonReleaseMask | Button3MotionMask);
    gc = XCreateGC(display, window, 0, NULL);

    XMapWindow(display, window);
    return true;
}

void windowRun(void (*drawCallback)(Display *, Window, GC, float))
{
    XEvent event;

    struct timespec prev, current;
    clock_gettime(CLOCK_MONOTONIC, &prev);

    int prevMouse_x = 0, prevMouse_y = 0;

    while (running)
    {
        while (XPending(display))
        {
            XNextEvent(display, &event);

            if (event.type == ClientMessage || event.type == DestroyNotify)
            {
                running = false;
            }

            if (event.type == ConfigureNotify)
            {
                windowH = event.xconfigure.height;
                windowW = event.xconfigure.width;
            }

            if (event.type == ButtonPress)
            {
                //Right click
                if (event.xbutton.button & Button3)
                {
                    prevMouse_x = event.xbutton.x;
                    prevMouse_y = event.xbutton.y;
                }

                //Scroll up
                if (event.xbutton.button == Button4){
                    camera.zoom += 0.25;
                }

                //Scroll down
                if (event.xbutton.button == Button5){
                    if (camera.zoom == 0.1){
                        continue;
                    }
                    camera.zoom -= 0.25;
                    if(camera.zoom < 0){
                        camera.zoom = 0.1;
                    }
                }
            }

            if (event.type == MotionNotify)
            {
                if (event.xmotion.state & Button3Mask)
                {
                    int dx = event.xmotion.x - prevMouse_x;
                    int dy = event.xmotion.y - prevMouse_y;
                    prevMouse_x = event.xmotion.x;
                    prevMouse_y = event.xmotion.y;

                    camera.pos.x += ((dx / camera.zoom) * CAMERA_MOVE_SPEED) * -1;
                    camera.pos.y += ((dy / camera.zoom) * CAMERA_MOVE_SPEED) * -1;
                }
            }
        }

        clock_gettime(CLOCK_MONOTONIC, &current);
        float deltaTime = (current.tv_sec - prev.tv_sec) +
                          (current.tv_nsec - prev.tv_nsec) / 1000000000.0f;
        prev = current;

        if (drawCallback)
        {
            drawCallback(display, window, gc, deltaTime);
        }

        usleep(16000); // ~60 FPS
    }
}

void windowClose()
{
    XFreeGC(display, gc);
    XDestroyWindow(display, window);
    XCloseDisplay(display);
}
