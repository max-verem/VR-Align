#ifndef calc_tracking_model_from_chess_results_h 
#define calc_tracking_model_from_chess_results_h 

#include "wt.h"

static void calc_tracking_model_from_chess_results
(
    const q_xyz_quat_type *arm,  /* rotation of mounted tracker and arm vector*/
    const q_xyz_quat_type *ref,  /* reference position, static tracker real position */
    std::vector<q_xyz_quat_type> &trks, /* montion tracker recalced */
    std::vector<q_xyz_quat_type> &cams  /* real camera optic center rotation and position */
)
{
    trks.resize(0);
    cams.resize(0);

    for (int i = 0; i < chess_results.size(); i++)
    {
        q_xyz_quat_type trk, cam, stk, mnt;

        // static tracker
        stk = chess_results[i]->vrpns[3].pose;

        // mounter tracker
        mnt = chess_results[i]->vrpns[2].pose;

        // calc model
        wt_calc_two_trackers_model_a(&stk, &mnt, arm, ref, &trk, &cam);

        // save results
        trks.push_back(trk);
        cams.push_back(cam);
    };
};
#endif
