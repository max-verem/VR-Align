#include <stdio.h>

#include <quat.h>
#include <math.h>
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

static void print_vec(const char* name, q_vec_type vec)
{
    printf("%s = [%f, %f, %f]\n", name, vec[0], vec[1], vec[2]);
}

static void vec2screen(q_vec_type screen, const q_vec_type vec, const double f)
{
    screen[0] = vec[1] * f / vec[0];
    screen[1] = vec[2] * f / vec[0];
    screen[2] = 0;
}

static void ref2screen(q_vec_type scr, const q_type rot_quat, const q_vec_type cam_ue4, const q_vec_type ref_ue4, const double focus_distance)
{
    // create vec
    q_vec_type dst;
    q_vec_subtract(dst, ref_ue4, cam_ue4);
//    print_vec(__FUNCTION__ "::dst", dst);

    // rotate dst vec
    q_vec_type dst_r;
    q_xform(dst_r, rot_quat, dst);
//    print_vec(__FUNCTION__ "::dst_r", dst_r);

    // translate to screen
    vec2screen(scr, dst_r, focus_distance);
//    print_vec(__FUNCTION__ "::scr", scr);
}

void perspective_projection_calc
(
    const double focus_distance,
    const double cam_ue4_x, const double cam_ue4_y, const double cam_ue4_z,
    const double cam_ue4_roll, const double cam_ue4_pitch, const double cam_ue4_yaw,
    const int cnt, q_vec_type *src, q_vec_type *dst
)
{
    int i;
    q_type rot_quat;
    q_vec_type cam_ue4;

    q_vec_set(cam_ue4, cam_ue4_x, cam_ue4_y, cam_ue4_z);

    // create quaterion of current camera rotation
    // NB: Z-Axis is 
    q_from_euler(rot_quat, Q_DEG_TO_RAD(-cam_ue4_yaw), Q_DEG_TO_RAD(cam_ue4_pitch), Q_DEG_TO_RAD(cam_ue4_roll));
///    printf("rot_quat = [%f, %f, %f, %f]\n", rot_quat[0], rot_quat[1], rot_quat[2], rot_quat[3]);

    // do for all incoming objects
    for (i = 0; i < cnt; i++)
    {
        q_vec_type scr;
        q_vec_type ref_ue4;

        q_vec_copy(ref_ue4, src[i]);
        ref2screen(scr, rot_quat, cam_ue4, ref_ue4, focus_distance);
        q_vec_copy(dst[i], scr);
    }
}

const double
    cam_ue4_yaw = 5.0,      // yaw is about Z
    cam_ue4_pitch = 4.0,    // pitch is rotation about Y
    cam_ue4_roll = 0.0;     // roll is rotation about X

const double
    cam_ue4_x = -184,
    cam_ue4_y = 0,
    cam_ue4_z = 107;

const double fx =
    1920 /* screen width */
    *
    35 /* focal lengt */
    /
    23.76 /* sensor width */; /* btw of sensor was height = 13.365 */

#define MAX_REFS 32

static const struct
{
    const char* title;
    double x, y, z;
} refs[] =
{
    { "12", 10, 10, 120},
    { "11", 0, 10, 140},
    { "10", 0, 60, 120},
    { "9", 0, 50, 130},
    { "8", 0, 30, 130},
    { "7", 0, 50, 110},
    { "6", 0, 20, 110},
    { "5", 0, 0, 110},
    { "4", 0, 70, 105},
    { "3", 0, 40, 98},
    { "2", 0, 23, 95},
    { "1", 0, 0, 100},

    { NULL }
};

int main()
{
    int i, j, cnt, m;
    q_vec_type REF_3D[MAX_REFS], REF_2D[MAX_REFS], CURR_2D[MAX_REFS];

    // setup refs
    for (i = 0, cnt = 0; refs[i].title; i++, cnt++)
        q_vec_set(REF_3D[i], refs[i].x, refs[i].y, refs[i].z);

    // calc refs
    perspective_projection_calc(fx, cam_ue4_x, cam_ue4_y, cam_ue4_z,
        cam_ue4_roll, cam_ue4_pitch, cam_ue4_yaw, cnt, REF_3D, REF_2D);

    // dump data
    for (i = 0; i < cnt; i++)
        printf("%s [%f, %f, %f] => [%f, %f]\n",
            refs[i].title, REF_3D[i][0], REF_3D[i][1], REF_3D[i][2], REF_2D[i][0], REF_2D[i][1]);

    // calc approx
    double
        c_cam_ue4_roll = cam_ue4_roll - 3.0,
        c_cam_ue4_pitch = cam_ue4_pitch - 3.0,
        c_cam_ue4_yaw = cam_ue4_yaw - 3.0;

    double M = -1.0, m_cam_ue4_pitch = 0, m_cam_ue4_yaw = 0, m_cam_ue4_roll = 0;

#define VAR_STEPS 3
#define VAR_COUNT 3
#define VAR_ROUND (VAR_STEPS * VAR_STEPS * VAR_STEPS)
    const double steps[VAR_STEPS] = { -0.1, 0.0, 0.1 };
    double steps_rounds[VAR_ROUND][VAR_COUNT];

    int round_zero = -1;
    for (j = 0; j < VAR_ROUND; j++)
    {
        double s = 0;
        for (i = 0; i < VAR_COUNT; i++)
        {
            double step = steps[(j / (int)powl(VAR_STEPS, i)) % VAR_STEPS];
            steps_rounds[j][i] = step;
            s += fabs(step);
        };
        if (!s)
            round_zero = j;
    }
    printf("round zero = %d\n", round_zero);

    while(1)
    {
        double dist[MAX_REFS], maxs[VAR_ROUND];

        for (j = 0; j < VAR_ROUND; j++)
        {
            // calc new projects
            perspective_projection_calc(fx, cam_ue4_x, cam_ue4_y, cam_ue4_z,
                c_cam_ue4_roll  + steps_rounds[j][0],
                c_cam_ue4_pitch + steps_rounds[j][1],
                c_cam_ue4_yaw   + steps_rounds[j][2],
                cnt, REF_3D, CURR_2D);

            // find distances
            for (i = 0; i < cnt; i++)
                dist[i] = q_vec_distance(REF_2D[i], CURR_2D[i]);
            // find maximal distance
            for (m = 0, i = 1; i < cnt; i++)
                if (dist[m] < dist[i])
                    m = i;
            // save
            maxs[j] = dist[m];
        };

        // find minimal distance
        for (m = 0, j = 1; j < VAR_ROUND; j++)
            if (maxs[m] > maxs[j])
                m = j;

        // update search vector
        c_cam_ue4_roll  += steps_rounds[m][0];
        c_cam_ue4_pitch += steps_rounds[m][1];
        c_cam_ue4_yaw   += steps_rounds[m][0];

        printf("{%d}[%f] => c_cam_ue4_roll=%f, c_cam_ue4_pitch=%f, c_cam_ue4_yaw=%f\n",
            m, maxs[m], c_cam_ue4_roll, c_cam_ue4_pitch, c_cam_ue4_yaw);

        if (m == round_zero)
            break;
    }

    return 0;
}
