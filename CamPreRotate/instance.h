#ifndef INSTANCE_H
#define INSTANCE_H

#include <Windows.h>
#include <quat.h>
#include <vrpn_Connection.h>
#include <vrpn_Tracker.h>

enum
{
    ARM_O_POS = 0,
    ARM_C_POS,
    ARM_L_POS,
    ARM_R_POS,
    ARM_U_POS,
    ARM_D_POS,
    ARM_LAST_POS
};

typedef struct instance_desc
{
    /* runtime */
    HANDLE lock;
    HANDLE vrpn_th;
    vrpn_Tracker_Remote *vrpn_tkr;
    q_vec_type armX_poses[ARM_LAST_POS];
    q_vec_type armX_intercepts[ARM_LAST_POS];
    q_vec_type armX_proj_px[ARM_LAST_POS];
    q_vec_type prerot;
    HWND hWnd;

    /* arguments */
    char *vrpn_address;
    q_vec_type camera_arm;
    q_vec_type v_screen_pos, v_screen_size;
    int window_pos[2], window_size[2];
    double frustum_deep, frustum_radius;

} instance_h;

#endif