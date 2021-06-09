#ifndef INSTANCE_H
#define INSTANCE_H

#include <Windows.h>

typedef struct instance_desc
{
    /* runtime */
    HANDLE lock;
    HANDLE vrpn_th;
    HWND hWnd;

    /* arguments */
    char *vrpn_address;
    int window_pos[2], window_size[2], chess_board_offset[2], chess_board_dim[2], chess_board_step;

} instance_h;

#endif