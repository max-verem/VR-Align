#include <stdio.h>

#include <quat.h>
#include <vrpn_Connection.h>
#include <vrpn_Tracker.h>

#pragma comment(lib, "quatlib.lib")
#pragma comment(lib, "vrpn.lib")

static vrpn_TRACKERCB first_data;
static int first_flag = 1;

static void VRPN_CALLBACK vrpn_handle_tracker(void* userData, const vrpn_TRACKERCB t)
{
#if 0
    q_vec_type pos;
    q_type quat;

    pos[0] = t.pos[0];
    pos[1] = t.pos[1];
    pos[2] = t.pos[2];

    quat[0] = t.quat[0];
    quat[1] = t.quat[1];
    quat[2] = t.quat[2];
    quat[3] = t.quat[3];
#else
    if (first_flag)
    {
        first_flag = 0;
        first_data = t;
    }

    printf("%8.6f,%8.6f,%8.6f\n",
        t.pos[0]/* - first_data.pos[0]*/,
        t.pos[1]/* - first_data.pos[1]*/,
        t.pos[2]/* - first_data.pos[2]*/);
#endif
};


int main(int argc, char** argv)
{
    vrpn_Tracker_Remote *vrpn_trk;

    if (argc < 2)
    {
        fprintf(stderr, "Error! Please specify vrpn address\n");
        exit(1);
    }

    vrpn_trk = new vrpn_Tracker_Remote(argv[1]);
    vrpn_trk->register_change_handler(NULL, vrpn_handle_tracker);

    printf("X,Y,Z\n");

    while (1) vrpn_trk->mainloop();

    return 0;
}
