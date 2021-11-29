#include <stdio.h>

#include <quat.h>
#include <vrpn_Connection.h>
#include <vrpn_Tracker.h>

#pragma comment(lib, "quatlib.lib")
#pragma comment(lib, "vrpn.lib")

static vrpn_TRACKERCB first_data;
static int first_flag = 1;
static q_xyz_quat_type prev;

static void VRPN_CALLBACK vrpn_handle_tracker(void* userData, const vrpn_TRACKERCB t)
{
    if (first_flag)
    {
        first_flag = 0;
        first_data = t;
    }
    else
    {
        q_type q;
        q_xyz_quat_type delta;
        q_vec_type yawPitchRoll;

        q_vec_subtract(delta.xyz, t.pos, prev.xyz);
        q_invert(q, prev.quat);
        q_mult(delta.quat, t.quat, q);

        q_to_euler(yawPitchRoll, delta.quat);

        printf
        (
            "%8.6f,%8.6f,%8.6f,%8.6f,%8.6f,%8.6f,%8.6f,%8.6f,%8.6f,%8.6f,%8.6f,%8.6f,%8.6f,%8.6f,%8.6f,%8.6f,%8.6f\n",

            t.pos[0],
            t.pos[1],
            t.pos[2],

            t.quat[0],
            t.quat[1],
            t.quat[2],
            t.quat[3],

            1000.0 * delta.xyz[0],
            1000.0 * delta.xyz[1],
            1000.0 * delta.xyz[2],

            delta.quat[0],
            delta.quat[1],
            delta.quat[2],
            delta.quat[3],

            Q_RAD_TO_DEG(yawPitchRoll[0]),
            Q_RAD_TO_DEG(yawPitchRoll[1]),
            Q_RAD_TO_DEG(yawPitchRoll[2])
        );
    };

    q_copy(prev.quat, t.quat);
    q_vec_copy(prev.xyz, t.pos);
};


int main(int argc, char** argv)
{
    vrpn_Tracker_Remote *vrpn_trk;

    if (argc < 2)
    {
        fprintf(stderr, "Error! Please specify vrpn address\n");
        exit(1);
    }

    memset(&prev, 0, sizeof(prev));

    vrpn_trk = new vrpn_Tracker_Remote(argv[1]);
    vrpn_trk->register_change_handler(NULL, vrpn_handle_tracker);

    printf("PX,PY,PZ,QX,QY,QZ,QW,dPX,dPY,dPZ,dQX,dQY,dQZ,dQW,dYaw,dPitch,dRoll\n");

    while (1) vrpn_trk->mainloop();

    return 0;
}
