#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <quat.h>

#pragma comment(lib, "quatlib.lib")

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

#ifdef _DEBUG
#pragma comment(lib, "opencv_world453d.lib")
#else
#pragma comment(lib, "opencv_world453.lib")
#endif

#include "../common/print_vecs.h"
#include "../common/wt.h"
#include "../common/chess_results.h"
#include "../common/ARG_CHECK.h"
#include "../common/calc_tracking_model_from_chess_results.h"
#include "../common/avg_xyz_quat.h"

static double avg_quats_diff
(
    std::vector<q_xyz_quat_type> &rot_diffs,
    q_xyz_quat_type &avg,
    bool verbose
)
{
    double l = 0;
    q_xyz_quat_type dst;

    avg_xyz_quat_v1(rot_diffs, &dst);

    avg = dst;

    if (verbose)
        print_xyz_quat(__FUNCTION__ ":", &dst);

    return sqrt(1 - dst.quat[3] * dst.quat[3]);
};

static void calc_quats_diff
(
    const q_xyz_quat_type *arm,
    const q_xyz_quat_type *ref,
    std::vector<q_xyz_quat_type> &trks,
    std::vector<q_xyz_quat_type> &cams,
    std::vector<q_xyz_quat_type> &rot_diffs,
    bool verbose
)
{
    int i;

    rot_diffs.clear();

    calc_tracking_model_from_chess_results(arm, ref, trks, cams);
    for (i = 0; i < chess_results.size(); i++)
    {
        // quat diff
        q_xyz_quat_type rot_diff, tmp;
        q_invert(tmp.quat, cams[i].quat);
        q_mult(rot_diff.quat, chess_results[i]->pose.quat, tmp.quat);

        q_vec_subtract(rot_diff.xyz, chess_results[i]->pose.xyz, cams[i].xyz);

        rot_diffs.push_back(rot_diff);

        if (verbose)
        {
            printf("\n");
            print_quat2("     cv", chess_results[i]->pose.quat);
            print_quat2("    cam", cams[i].quat);
            print_quat2("    trk", trks[i].quat);
            print_quat2("\t\trot_diff", rot_diff.quat);
        }
    }
};

static void dir_vec_build(int index, q_vec_type dir, double step)
{
    int i;
    double steps[3] = { 0, step, -step };

    for (i = 0; i < 3; i++)
    {
        dir[i] = steps[index % 3];
        index /= 3;
    }
}

static int task_find_pos(q_xyz_quat_type *arm, q_xyz_quat_type *ref)
{
    int i, j;
    double step = 0.001, best = 10000000;
    q_vec_type ref_curr = { 1.0, 1.0, 1.0 };
    std::vector<q_xyz_quat_type> trks, cams;

    while (1)
    {
        int mn;
        q_vec_type ref_dir;
        double avgs[27];
        q_xyz_quat_type new_arm_avg[27];

        for (i = 1; i < 27; i++)
        {
            dir_vec_build(i % 27, ref_dir, step);
            q_vec_add(ref_dir, ref_dir, ref_curr);

            q_vec_copy(ref->xyz, ref_dir);
            cams.clear(); trks.clear();
            ///                printf("calc_tracking_model_from_chess_results[%d]...", i);
            calc_tracking_model_from_chess_results(arm, ref, trks, cams);
            ///                printf("\n");

            std::vector<q_xyz_quat_type> new_arms;
            for (j = 0; j < chess_results.size(); j++)
            {
                q_xyz_quat_type new_arm;

                q_vec_subtract(new_arm.xyz, chess_results[j]->pose.xyz, cams[j].xyz);
                q_copy(new_arm.quat, chess_results[j]->pose.quat);
                q_xform(new_arm.xyz, new_arm.quat, new_arm.xyz);
                if (j == 0)
                    new_arm_avg[i] = new_arm;
                else
                    q_vec_add(new_arm_avg[i].xyz, new_arm_avg[i].xyz, new_arm.xyz);
                new_arms.push_back(new_arm);
            }
            q_vec_scale(new_arm_avg[i].xyz, 1.0 / chess_results.size(), new_arm_avg[i].xyz);
            for (avgs[i] = 0, j = 0; j < chess_results.size(); j++)
                avgs[i] += q_vec_distance(new_arm_avg[i].xyz, new_arms[j].xyz);
        };

        for (i = 2, mn = 1; i < 27; i++)
            if (avgs[mn] > avgs[i])
                mn = i;

        dir_vec_build(mn % 27, ref_dir, step);
        q_vec_add(ref_curr, ref_dir, ref_curr);

        printf("\t%f\n", avgs[mn] / chess_results.size());
        print_vec("\t\tref.xyz", ref_curr);
        print_vec("\t\tarm.xyz", new_arm_avg[mn].xyz);

        if (avgs[mn] >= best)
        {
            q_vec_copy(ref->xyz, ref_curr);
            q_vec_copy(arm->xyz, new_arm_avg[mn].xyz);
            return 0;
        }

        best = avgs[mn];
    }

    return -1;
}

static double task_find_pos2(q_xyz_quat_type *arm, q_xyz_quat_type *ref, int f_criteria_maximal)
{
    int i, j;
    q_vec_type arm_curr, ref_curr;
    double step = 0.001, best = 10000000;
    std::vector<q_xyz_quat_type> trks, cams;

    q_vec_copy(arm_curr, arm->xyz);
    q_vec_copy(ref_curr, ref->xyz);

    while (1)
    {
        int mn;
        double avgs[27 * 27];
        q_vec_type arm_dir, ref_dir;

        for (i = 1; i < 27 * 27; i++)
        {
            dir_vec_build(i % 27, arm_dir, step);
            q_vec_add(arm_dir, arm_dir, arm_curr);

            dir_vec_build(i / 27, ref_dir, step);
            q_vec_add(ref_dir, ref_dir, ref_curr);

            q_vec_copy(arm->xyz, arm_dir);
            q_vec_copy(ref->xyz, ref_dir);

            cams.clear(); trks.clear();
            calc_tracking_model_from_chess_results(arm, ref, trks, cams);

            for (avgs[i] = 0, j = 0; j < chess_results.size(); j++)
            {
                double d = q_vec_distance(chess_results[j]->pose.xyz, cams[j].xyz);

                if (f_criteria_maximal)
                {
                    if (avgs[i] < d)
                        avgs[i] = d;
                }
                else
                    avgs[i] += d / chess_results.size();
            }
        };

        for (i = 2, mn = 1; i < 27 * 27; i++)
            if (avgs[mn] > avgs[i])
                mn = i;

        dir_vec_build(mn % 27, arm_dir, step);
        q_vec_add(arm_curr, arm_dir, arm_curr);

        dir_vec_build(mn / 27, ref_dir, step);
        q_vec_add(ref_curr, ref_dir, ref_curr);

        printf("\t%f\n", avgs[mn]);
        print_vec("\t\tref.xyz", ref_curr);
        print_vec("\t\tarm.xyz", arm_curr);

        if (avgs[mn] >= best)
        {
            q_vec_copy(arm->xyz, arm_curr);
            q_vec_copy(ref->xyz, ref_curr);
            return avgs[mn];
        }

        best = avgs[mn];
    }

    return -1;
}


static double task_find_arm_pos(q_xyz_quat_type *arm, q_xyz_quat_type *ref)
{
    int i, j;
    q_vec_type arm_curr;
    double step = 0.001, best = 10000000;
    std::vector<q_xyz_quat_type> trks, cams;

    q_vec_copy(arm_curr, arm->xyz);

    while (1)
    {
        int mn;
        double avgs[27];
        q_vec_type arm_dir;

        for (i = 1; i < 27; i++)
        {
            dir_vec_build(i % 27, arm_dir, step);
            q_vec_add(arm_dir, arm_dir, arm_curr);

            q_vec_copy(arm->xyz, arm_dir);
            cams.clear(); trks.clear();
            calc_tracking_model_from_chess_results(arm, ref, trks, cams);

            for (avgs[i] = 0, j = 0; j < chess_results.size(); j++)
                avgs[i] += q_vec_distance(chess_results[j]->pose.xyz, cams[j].xyz);
        };

        for (i = 2, mn = 1; i < 27; i++)
            if (avgs[mn] > avgs[i])
                mn = i;

        dir_vec_build(mn % 27, arm_dir, step);
        q_vec_add(arm_curr, arm_dir, arm_curr);

        printf("\t%f\n", avgs[mn] / chess_results.size());
        print_vec("\t\tarm.xyz", arm_curr);

        if (avgs[mn] >= best)
        {
            q_vec_copy(arm->xyz, arm_curr);
            return avgs[mn];
        }

        best = avgs[mn];
    }

    return -1;
}

static double task_find_ref_pos(q_xyz_quat_type *arm, q_xyz_quat_type *ref)
{
    int i, j;
    q_vec_type ref_curr;
    double step = 0.001, best = 10000000;
    std::vector<q_xyz_quat_type> trks, cams;

    q_vec_copy(ref_curr, ref->xyz);

    while (1)
    {
        int mn;
        double avgs[27];
        q_vec_type ref_dir;

        for (i = 1; i < 27; i++)
        {
            dir_vec_build(i % 27, ref_dir, step);
            q_vec_add(ref_dir, ref_dir, ref_curr);

            q_vec_copy(ref->xyz, ref_dir);
            cams.clear(); trks.clear();
            calc_tracking_model_from_chess_results(arm, ref, trks, cams);

            for (avgs[i] = 0, j = 0; j < chess_results.size(); j++)
                avgs[i] += q_vec_distance(chess_results[j]->pose.xyz, cams[j].xyz);
        };

        for (i = 2, mn = 1; i < 27; i++)
            if (avgs[mn] > avgs[i])
                mn = i;

        dir_vec_build(mn % 27, ref_dir, step);
        q_vec_add(ref_curr, ref_dir, ref_curr);

        printf("\t%f\n", avgs[mn] / chess_results.size());
        print_vec("\t\tref.xyz", ref_curr);

        if (avgs[mn] >= best)
        {
            q_vec_copy(ref->xyz, ref_curr);
            return 0;
        }

        best = avgs[mn];
    }

    return -1;
}


static int task_find_arm_rot(q_xyz_quat_type *arm, q_xyz_quat_type *ref)
{
    q_xyz_quat_type avg_arm;
    std::vector<q_xyz_quat_type> trks, cams, rot_diffs;

    calc_quats_diff(arm, ref, trks, cams, rot_diffs, 0);
    avg_quats_diff(rot_diffs, avg_arm, 1);

    q_copy(arm->quat, avg_arm.quat);

    return 0;
}

static int task_find_ref_rot(q_xyz_quat_type *arm, q_xyz_quat_type *ref)
{
    q_xyz_quat_type avg_ref;
    std::vector<q_xyz_quat_type> trks, cams, rot_diffs;

    calc_quats_diff(arm, ref, trks, cams, rot_diffs, 0);
    avg_quats_diff(rot_diffs, avg_ref, 1);

    q_copy(ref->quat, avg_ref.quat);

    return 0;
}

static int task_find_ref_rot_first(q_xyz_quat_type *arm, q_xyz_quat_type *ref)
{
    int i;
    double step = 0.001, best = 100000;
    q_vec_type ref_curr = { 0.0, 0.0, 0.0 };
    std::vector<q_xyz_quat_type> trks, cams;

    while (1)
    {
        int mn;
        q_vec_type ref_dir;
        double avgs[27];

        for (i = 1; i < 27; i++)
        {
            q_xyz_quat_type tmp;
            std::vector<q_xyz_quat_type> rot_diffs;

            dir_vec_build(i, ref_dir, step);
            q_vec_add(ref_dir, ref_dir, ref_curr);

            q_from_euler(ref->quat, Q_DEG_TO_RAD(ref_dir[2]), Q_DEG_TO_RAD(ref_dir[1]), Q_DEG_TO_RAD(ref_dir[0]));

            calc_quats_diff(arm, ref, trks, cams, rot_diffs, 0);
            avgs[i] = avg_quats_diff(rot_diffs, tmp, 0);
        };

        for (i = 2, mn = 1; i < 27; i++)
            if (avgs[mn] > avgs[i])
                mn = i;

        dir_vec_build(mn, ref_dir, step);
        q_vec_add(ref_curr, ref_dir, ref_curr);

        printf("\t(Yaw=%f',Roll=%f',Pitch=%f'), avg=%f\n", ref_curr[2], ref_curr[1], ref_curr[0], avgs[mn]);

        if (best < avgs[mn])
        {
            q_from_euler(ref->quat, Q_DEG_TO_RAD(ref_curr[2]), Q_DEG_TO_RAD(ref_curr[1]), Q_DEG_TO_RAD(ref_curr[0]));
            return 0;
        }

        best = avgs[mn];
    }

    return -1;
}

static int task_find_arm_rot_first(q_xyz_quat_type *arm, q_xyz_quat_type *ref)
{
    int i;
    double step = 0.001, best = 100000;
    q_vec_type arm_curr = { 0.0, 0.0, 0.0 };
    std::vector<q_xyz_quat_type> trks, cams;

    while (1)
    {
        int mn;
        q_vec_type arm_dir;
        double avgs[27];

        for (i = 1; i < 27; i++)
        {
            q_xyz_quat_type tmp;
            std::vector<q_xyz_quat_type> rot_diffs;

            dir_vec_build(i, arm_dir, step);
            q_vec_add(arm_dir, arm_dir, arm_curr);

            q_from_euler(arm->quat, Q_DEG_TO_RAD(arm_dir[2]), Q_DEG_TO_RAD(arm_dir[1]), Q_DEG_TO_RAD(arm_dir[0]));

            calc_quats_diff(arm, ref, trks, cams, rot_diffs, 0);
            avgs[i] = avg_quats_diff(rot_diffs, tmp, 0);
        };

        for (i = 2, mn = 1; i < 27; i++)
            if (avgs[mn] > avgs[i])
                mn = i;

        dir_vec_build(mn, arm_dir, step);
        q_vec_add(arm_curr, arm_dir, arm_curr);

        printf("\t(Yaw=%f',Roll=%f',Pitch=%f'), avg=%f\n", arm_curr[2], arm_curr[1], arm_curr[0], avgs[mn]);

        if (best < avgs[mn])
        {
            q_from_euler(arm->quat, Q_DEG_TO_RAD(arm_curr[2]), Q_DEG_TO_RAD(arm_curr[1]), Q_DEG_TO_RAD(arm_curr[0]));
            return 0;
        }

        best = avgs[mn];
    }

    return -1;
}

static int task_cam_diff(q_xyz_quat_type *arm, q_xyz_quat_type *ref)
{
    int i;
    q_vec_type vec0 = Q_NULL_VECTOR;
    std::vector<q_xyz_quat_type> trks, cams;
    calc_tracking_model_from_chess_results(arm, ref, trks, cams);

    for (i = 0; i < chess_results.size(); i++)
    {
        q_xyz_quat_type diff;

        q_vec_subtract(diff.xyz, chess_results[i]->pose.xyz, cams[i].xyz);
        q_invert(diff.quat, cams[i].quat);
        q_mult(diff.quat, chess_results[i]->pose.quat, diff.quat);

        //            printf("[%4d]\n", i);
        //            print_xyz_quat("    trk", &trks[i]);
        //            print_xyz_quat("     cv", &chess_results[i]->pose);
        //            print_xyz_quat("    cam", &cams[i]);
        printf("%5.3f ", q_vec_distance(vec0, diff.xyz));
        print_xyz_quat("   diff", &diff);
        //            printf("\n");
    };

    return 0;
}

static void q_from_euler_grad(q_type q, const double yaw, const double pitch, const double roll)
{
    q_from_euler(q, Q_DEG_TO_RAD(yaw), Q_DEG_TO_RAD(pitch), Q_DEG_TO_RAD(roll));
};

static void xyz_from_euler_grad(q_xyz_quat_type* p, const double x, const double y, const double z, const double yaw, const double pitch, const double roll)
{
    q_from_euler_grad(p->quat, yaw, pitch, roll);
    p->xyz[0] = x;
    p->xyz[1] = y;
    p->xyz[2] = z;
};

int main(int argc, char** argv)
{
    int i, j;

    q_vec_type vec0 = Q_NULL_VECTOR;
    static const q_type prerot0 = Q_ID_QUAT;

    q_xyz_quat_type arm;
    q_vec_copy(arm.xyz, vec0);
    q_copy(arm.quat, prerot0);

    q_xyz_quat_type ref;
    q_vec_copy(ref.xyz, vec0);
    q_copy(ref.quat, prerot0);

    char
        *task = NULL,
        parse_chess_board_yaml[4096] = "D:\\projects\\VR-Align\\_2021-06-21.chess\\ParseChessBoard-1628434980.yaml";

    // check/set arguments
    for (i = 1; i < argc;)
    {
        ARG_CHECK_BEGIN("parse-chess-board-yaml", 1)
            strcpy(parse_chess_board_yaml, argv[i + 1]);
        ARG_CHECK_END(1)
        else
        ARG_CHECK_BEGIN("task", 1)
            task = argv[i + 1];
        ARG_CHECK_END(1)
        else
        ARG_CHECK_BEGIN("ref-pos", 3)
            ref.xyz[0] = atof(argv[i + 1]);
            ref.xyz[1] = atof(argv[i + 2]);
            ref.xyz[2] = atof(argv[i + 3]);
        ARG_CHECK_END(3)
        else
        ARG_CHECK_BEGIN("arm-pos", 3)
            arm.xyz[0] = atof(argv[i + 1]);
            arm.xyz[1] = atof(argv[i + 2]);
            arm.xyz[2] = atof(argv[i + 3]);
        ARG_CHECK_END(3)
        else
        ARG_CHECK_BEGIN("ref-quat", 4)
            ref.quat[0] = atof(argv[i + 1]);
            ref.quat[1] = atof(argv[i + 2]);
            ref.quat[2] = atof(argv[i + 3]);
            ref.quat[2] = atof(argv[i + 3]);
        ARG_CHECK_END(4)
        else
        ARG_CHECK_BEGIN("ref-euler", 3)
            q_from_euler(ref.quat,
                Q_DEG_TO_RAD(atof(argv[i + 1])),
                Q_DEG_TO_RAD(atof(argv[i + 2])),
                Q_DEG_TO_RAD(atof(argv[i + 3])));
        ARG_CHECK_END(3)
        else
        ARG_CHECK_BEGIN("arm-quat", 4)
            arm.quat[0] = atof(argv[i + 1]);
            arm.quat[1] = atof(argv[i + 2]);
            arm.quat[2] = atof(argv[i + 3]);
            arm.quat[2] = atof(argv[i + 3]);
        ARG_CHECK_END(4)
        else
        ARG_CHECK_BEGIN("arm-euler", 3)
            q_from_euler(arm.quat,
                Q_DEG_TO_RAD(atof(argv[i + 1])),
                Q_DEG_TO_RAD(atof(argv[i + 2])),
                Q_DEG_TO_RAD(atof(argv[i + 3])));
        ARG_CHECK_END(3)
        else
        {
            fprintf(stderr, "Error! Failed to parse arg #%d [%s]\n", i, argv[i]);
            exit(1);
        };
    };

    // load sources
    chess_results_load(parse_chess_board_yaml);

    print_xyz_quat("REF", &ref);
    print_xyz_quat("ARM", &arm);

    // check which task specified
    if (!task)
    {
        printf("executing [find_arm_rot_first]\n");
        task_find_arm_rot_first(&arm, &ref);
        print_xyz_quat("REF", &ref);
        print_xyz_quat("ARM", &arm);

        printf("executing [find_ref_rot]\n");
        task_find_ref_rot(&arm, &ref);
        print_xyz_quat("REF", &ref);
        print_xyz_quat("ARM", &arm);

        printf("--------------------- rotation differences ----------------------------------------\n");
        {
            std::vector<q_xyz_quat_type> rot_diffs, trks, cams;
            calc_quats_diff(&arm, &ref, trks, cams, rot_diffs, 1);
        }
        printf("-----------------------------------------------------------------------------------\n");
#if 0
        printf("executing [find_pos]\n");
        task_find_pos(&arm, &ref);
        print_xyz_quat("REF", &ref);
        print_xyz_quat("ARM", &arm);

        printf("--------------------- cameras differences ----------------------------------------\n");
        task_cam_diff(&arm, &ref);
        printf("----------------------------------------------------------------------------------\n");
#endif
        printf("executing [find_pos2]\n");
        task_find_pos2(&arm, &ref, 1);
        print_xyz_quat("REF", &ref);
        print_xyz_quat("ARM", &arm);

        printf("--------------------- cameras differences ----------------------------------------\n");
        task_cam_diff(&arm, &ref);
        printf("----------------------------------------------------------------------------------\n");

#if 0
        // just print what is here
        calc_tracking_model_from_chess_results(&arm, &ref, trks, cams);
        for (i = 0; i < chess_results.size(); i++)
        {
            printf("\n");
            print_vec("     cv", chess_results[i]->pose.xyz);
            print_quat2("\t\t", chess_results[i]->pose.quat);
            print_vec("    cam", cams[i].xyz);
            print_quat2("\t\t", cams[i].quat);
            print_vec("    trk", trks[i].xyz);
            print_quat2("\t\t", trks[i].quat);
        }
#endif
    }
    else if (!strcmp("dist_diff", task))
    {
        for (j = 0; j < chess_results.size(); j++)
        {
            double t = 0;
            for (i = 0; i < chess_results.size(); i++)
            {
                double
                    d,
                    diff0 = q_vec_distance(chess_results[i]->vrpns[2].pose.xyz, chess_results[j]->vrpns[2].pose.xyz),
                    diff1 = q_vec_distance(chess_results[i]->pose.xyz, chess_results[j]->pose.xyz);

                d = fabs(diff0 - diff1);
                printf(" %6.3f | ", d);
                t += d;
                //printf("\t%f\t%f\t%f\n", diff0 - diff1, diff0, diff1);
            }
            printf(" [ %6.3f ] @ %s\n", t, chess_results[j]->filename);
        };
    }
    else if (!strcmp("rot_diff", task))
    {
        std::vector<q_xyz_quat_type> rot_diffs, trks, cams;
        calc_quats_diff(&arm, &ref, trks, cams, rot_diffs, 1);
    }
    else if (!strcmp("find_arm_rot", task))
    {
        task_find_arm_rot(&arm, &ref);
    }
    else if (!strcmp("find_ref_rot", task))
    {
        task_find_ref_rot_first(&arm, &ref);
    }
    else if (!strcmp("cam_diff", task))
    {
        task_cam_diff(&arm, &ref);
    }
    else if (!strcmp("find_pos", task))
    {
        task_find_pos(&arm, &ref);
    }
    else if (!strcmp("find_pos2", task))
    {
        task_find_pos2(&arm, &ref, 1);
    }
    else if (!strcmp("find_arm_pos", task))
    {
        task_find_arm_pos(&arm, &ref);
        print_xyz_quat("REF", &ref);
        print_xyz_quat("ARM", &arm);
    }
    else if (!strcmp("find_ref_pos", task))
    {
        task_find_ref_pos(&arm, &ref);
        print_xyz_quat("REF", &ref);
        print_xyz_quat("ARM", &arm);
    }
    else if (!strcmp("test1", task))
    {
        q_xyz_quat_type _stk, pos1, pos2;

        xyz_from_euler_grad(&ref,
            0.1, 0.2, 1.51,
            0.0, 0.0, 0.0);

        xyz_from_euler_grad(&_stk,
            -0.0287, -0.7474, -0.5767,
            -179.7824, -9.5294, 178.4949);

        xyz_from_euler_grad(&pos1,
            -0.0062, -0.7430, -0.6816,
            -179.4741, 2.6124, 179.4423);

        xyz_from_euler_grad(&pos2,
            -0.0052, -0.7438, -0.6825,
            -179.5517, -1.9056, 179.4723);

        print_xyz_quat("ref", &ref);
        print_xyz_quat("_stk", &_stk);

        for (i = 0; i < 100; i++)
        {
            q_xyz_quat_type _mnt;

            for (j = 0; j < 3; j++)
                _mnt.xyz[j] = pos1.xyz[j] + (pos2.xyz[j] - pos1.xyz[j]) * i / 100.0;

            q_slerp(_mnt.quat, pos1.quat, pos2.quat, i / 100.0);

            printf("\n");
            print_xyz_quat("_mnt", &_mnt);

            q_xyz_quat_type stk, mnt, trk, cam;

            wt_openvr_to_ue(&_stk, &stk);
            wt_openvr_to_ue(&_mnt, &mnt);

            print_xyz_quat("\tstk", &stk);
            print_xyz_quat("\tmnt", &mnt);

            wt_calc_two_trackers_model_a(&stk, &mnt, &arm, &ref, &trk, &cam);

            print_xyz_quat("\t\ttrk", &trk);
            print_xyz_quat("\t\tcam", &cam);
        }

//    print_xyz_quat("REF", &ref);
//    print_xyz_quat("ARM", &arm);
    }
    else
    {
        fprintf(stderr, "Error! Failed to recognize task [%s]\n", task);
        exit(0);
    };

    return 0;
};
