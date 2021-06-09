#include <stdio.h>

#include <quat.h>

#pragma comment(lib, "quatlib.lib")


/*****************************************************************************
 *
    q_from_euler - converts 3 euler angles (in radians) to a quaternion

   Assumes roll is rotation about X, pitch
   is rotation about Y, yaw is about Z.  Assumes order of
   yaw, pitch, roll applied as follows:

       p' = roll( pitch( yaw(p) ) )

      See comments for q_euler_to_col_matrix for more on this.
 *
 *****************************************************************************/


 // #define Q_DEG_TO_RAD(deg)       ( ((deg)*Q_PI)/180.0 )

const double
cam_ue4_yaw = 5.0,      // yaw is about Z
cam_ue4_pitch = 4.0,    // pitch is rotation about Y
cam_ue4_roll = 0.0;     // roll is rotation about X

const double
cam_ue4_x = -184,
cam_ue4_y = 0,
cam_ue4_z = 107;

const double
ref_ue4_x = 0,
ref_ue4_y = 0,
ref_ue4_z = 100;

const double fx =
    1920 /* screen width */
    *
    35 /* focal lengt */
    /
    23.76 /* sensor width */; /* btw of sensor was height = 13.365 */

static void print_matrix_4x4(const char* name, q_matrix_type matrix)
{
    printf
    (
        "%s = [\n"
        "    %f, %f, %f, %f\n"
        "    %f, %f, %f, %f\n"
        "    %f, %f, %f, %f\n"
        "    %f, %f, %f, %f\n"
        "]\n", name,
        matrix[0][0], matrix[0][1], matrix[0][2], matrix[0][3],
        matrix[1][0], matrix[1][1], matrix[1][2], matrix[1][3],
        matrix[2][0], matrix[2][1], matrix[2][2], matrix[2][3],
        matrix[3][0], matrix[3][1], matrix[3][2], matrix[3][3]
    );
}

static void zero_matrix(q_matrix_type m)
{
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) m[i][j] = 0.0;
}

int main()
{
    q_type rot_quat, rot_quat_i;
    q_matrix_type rot_matrix, E, K, EK, I, IEK;

    // create quaterion of current camera rotation
    q_from_euler(rot_quat, Q_DEG_TO_RAD(-cam_ue4_yaw), Q_DEG_TO_RAD(cam_ue4_pitch), Q_DEG_TO_RAD(cam_ue4_roll));
    printf("rot_quat = [%f, %f, %f, %f]\n", rot_quat[0], rot_quat[1], rot_quat[2], rot_quat[3]);

    // create rotation matrix
    q_to_col_matrix(rot_matrix, rot_quat);
    print_matrix_4x4("rot_matrix", rot_matrix);

    // setup E matrix
    // add t1, t2, t3 components
    // see https://neuralet.com/article/camera-calibration-using-homography-estimation/
    q_matrix_copy(E, rot_matrix);
    print_matrix_4x4("E", E);

    // setup K matrix
    zero_matrix(K);
    K[0][0] = ref_ue4_x - cam_ue4_x;
    K[1][0] = ref_ue4_y - cam_ue4_y;
    K[2][0] = ref_ue4_z - cam_ue4_z;
    print_matrix_4x4("K", K);

    // multiply matrix
    q_matrix_mult(EK, E, K);
    print_matrix_4x4("EK", EK);

    // setup K matrix
    zero_matrix(I);
    I[0][0] = 1;
    I[1][1] = fx;
    I[2][2] = fx;
    print_matrix_4x4("I", I);

    // multiply matrix
    q_matrix_mult(IEK, I, EK);
    print_matrix_4x4("IEK", IEK);

    IEK[1][0] /= IEK[0][0];
    IEK[2][0] /= IEK[0][0];
    IEK[0][0] /= IEK[0][0];
    print_matrix_4x4("IEK'", IEK);

    return 0;
}

