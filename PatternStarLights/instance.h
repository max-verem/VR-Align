#ifndef INSTANCE_H
#define INSTANCE_H

#include <Windows.h>
#include <quat.h>

#define MAX_STARS 1000

typedef struct instance_desc
{
    /* runtime */
    HANDLE lock;
    HANDLE vrpn_th;
    HWND hWnd;

    /* arguments */
    char *vrpn_address;
    int window_pos[2], window_size[2], stars_count, stars_radius, screen_sa, stars_sa;

    q_vec_type stars[MAX_STARS];

} instance_h;

#endif