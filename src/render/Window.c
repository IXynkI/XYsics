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
    XSelectInput(display, window, ExposureMask | KeyPressMask | StructureNotifyMask);
    gc = XCreateGC(display, window, 0, NULL);

    XMapWindow(display, window);
    return true;
}

void windowRun(void (*drawCallback)(Display *, Window, GC, float))
{
    XEvent event;

    struct timespec prev, current;
    clock_gettime(CLOCK_MONOTONIC, &prev);

    XSelectInput(display, window, StructureNotifyMask);


    while (running)
    {
        while (XPending(display))
        {
            XNextEvent(display, &event);

            if (event.type == ClientMessage || event.type == DestroyNotify)
            {
                running = false;
            }
        }

        clock_gettime(CLOCK_MONOTONIC, &current);
        float deltaTime = (current.tv_sec - prev.tv_sec) +
                          (current.tv_nsec - prev.tv_nsec) / 1000000000.0f;
        prev = current;

        if (event.type == ConfigureNotify){
            windowH = event.xconfigure.height;
            windowW = event.xconfigure.width;
        }

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
