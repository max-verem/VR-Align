#define _CRT_SECURE_NO_WARNINGS

#include <opencv2/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <iostream>

#ifdef _DEBUG
#pragma comment(lib, "opencv_world454d.lib")
#else
#pragma comment(lib, "opencv_world454.lib")
#endif

#define COUT std::cout

#include <quat.h>
#pragma comment(lib, "quatlib.lib")

#include "../common/wt.h"
#include "../common/wt_cv.h"
#include "../common/print_vecs.h"
#include "../common/ARG_CHECK.h"
#include "../common/avg_xyz_quat.h"

static void conv1(const cv::Mat& xyz, const cv::Mat& quat, q_xyz_quat_type *pos)
{
    pos->xyz[0] = xyz.at<double>(0, 0);
    pos->xyz[1] = xyz.at<double>(1, 0);
    pos->xyz[2] = xyz.at<double>(2, 0);

    pos->quat[0] = quat.at<double>(0, 0);
    pos->quat[1] = quat.at<double>(1, 0);
    pos->quat[2] = quat.at<double>(2, 0);
    pos->quat[3] = quat.at<double>(3, 0);
}

q_vec_type vec0 = Q_NULL_VECTOR;
static const q_type prerot0 = Q_ID_QUAT;

static std::vector<q_xyz_quat_type> CAMs, STKs, MNTs;

static void calc_tracking_model
(
    const q_xyz_quat_type* arm,  /* rotation of mounted tracker and arm vector*/
    const q_xyz_quat_type* ref,  /* reference position, static tracker real position */
    std::vector<q_xyz_quat_type>& trks, /* montion tracker recalced */
    std::vector<q_xyz_quat_type>& cams  /* real camera optic center rotation and position */
)
{
    trks.resize(0);
    cams.resize(0);

    for (int i = 0; i < CAMs.size(); i++)
    {
        q_xyz_quat_type trk, cam, stk, mnt;

        // static tracker
        stk = STKs[i];

        // mounted tracker
        mnt = MNTs[i];

        // calc model
        wt_calc_two_trackers_model_a(&stk, &mnt, arm, ref, &trk, &cam);

        // save results
        trks.push_back(trk);
        cams.push_back(cam);
    };
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

static double avg_quats_diff
(
    std::vector<q_xyz_quat_type>& rot_diffs,
    q_xyz_quat_type& avg,
    bool verbose = false
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

static void calc_tracking_model_diff
(
    const q_xyz_quat_type* arm,
    const q_xyz_quat_type* ref,
    std::vector<q_xyz_quat_type>& diffs,
    bool verbose = false
)
{
    int i;
    double d, dmin = 100000.0, dmax = -1.0, davg = 0;
    std::vector<q_xyz_quat_type> trks, cams;

    diffs.clear();

    calc_tracking_model(arm, ref, trks, cams);
    for (i = 0; i < CAMs.size(); i++)
    {
        // quat diff
        q_xyz_quat_type diff, tmp;
        q_invert(tmp.quat, cams[i].quat);
        q_mult(diff.quat, CAMs[i].quat, tmp.quat);

        q_vec_subtract(diff.xyz, CAMs[i].xyz, cams[i].xyz);

        diffs.push_back(diff);

        if (verbose)
        {
            printf("[%d]\n", i);
            print_xyz_quat("    CAM", &CAMs[i]);
            print_xyz_quat("    cam", &cams[i]);
            print_xyz_quat("    trk", &trks[i]);
            print_xyz_quat("   DIFF", &diff);

            d = q_vec_distance(diff.xyz, vec0);
            printf("       d = %f\n", d);

            if (d > dmax) dmax = d;
            if (d < dmin) dmin = d;
            davg += d;
        }
    }

    if (verbose)
        printf("dmin=%f, davg=%f, dmax=%f\n", dmin, davg / CAMs.size(), dmax);
};

static int task_find_arm_rot(q_xyz_quat_type* arm, q_xyz_quat_type* ref)
{
    int i;
    double step = 0.001, best = 100000;
    q_vec_type arm_curr = { 0.0, 0.0, 0.0 };

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

            calc_tracking_model_diff(arm, ref, rot_diffs);
            avgs[i] = avg_quats_diff(rot_diffs, tmp);
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

static double task_find_pos2(q_xyz_quat_type* arm, q_xyz_quat_type* ref, int f_criteria_maximal)
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
            calc_tracking_model(arm, ref, trks, cams);

            for (avgs[i] = 0, j = 0; j < CAMs.size(); j++)
            {
                double d = q_vec_distance(CAMs[j].xyz, cams[j].xyz);

                if (f_criteria_maximal)
                {
                    if (avgs[i] < d)
                        avgs[i] = d;
                }
                else
                    avgs[i] += d / CAMs.size();
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


/**
 */
int main(int argc, char *argv[]) {
    q_xyz_quat_type plane;
    char* img_path = NULL;
    int i, img_ses = 0, img_idx = 0, task_find_pos2_criteria = 0;

    // check/set arguments
    for (i = 1; i < argc;)
    {
        ARG_CHECK_BEGIN("path", 1)
            img_path = argv[i + 1];
        ARG_CHECK_END(1)
        else
        ARG_CHECK_BEGIN("ses", 1)
            img_ses = atoi(argv[i + 1]);
        ARG_CHECK_END(1)
        else
        ARG_CHECK_BEGIN("idx", 1)
            img_idx = atoi(argv[i + 1]);
        ARG_CHECK_END(1)
        else
        ARG_CHECK_BEGIN("plane", 7)
            plane.xyz[0] = atof(argv[i + 1]);
            plane.xyz[1] = atof(argv[i + 2]);
            plane.xyz[2] = atof(argv[i + 3]);
            plane.quat[0] = atof(argv[i + 4]);
            plane.quat[1] = atof(argv[i + 5]);
            plane.quat[2] = atof(argv[i + 6]);
            plane.quat[3] = atof(argv[i + 7]);
        ARG_CHECK_END(7)
        else
        ARG_CHECK_BEGIN("task_find_pos2_criteria", 1)
            task_find_pos2_criteria = atoi(argv[i + 1]);
        ARG_CHECK_END(1)
        else
        {
            fprintf(stderr, "Error! Failed to parse arg #%d [%s]\n", i, argv[i]);
            exit(1);
        };
    };

    print_xyz_quat("plane", &plane);

    cv::Mat PLANE;
    wt_quat_xyz_to_Mat4(&plane, PLANE);
    COUT << "PLANE=" << PLANE << std::endl;

    /* read configuration */
    while (1) {

        cv::Mat V_cv, C_cv, VRPN_xyz, VRPN_quat;
        q_xyz_quat_type cam_ue, cam_cv, vrpn_stk, vrpn_mnt, vrpn_cam, pos;
        cv::FileStorage fs;
        char path[1024];

        COUT << "Loading: ";

        // create pose filename
        snprintf(path, sizeof(path), "%s\\detect_aruco_markers_%d_pose_%03d.yaml",
            img_path, img_ses, img_idx);

        COUT << path << "... ";

        // read pose result
        try
        {
            fs.open(path, cv::FileStorage::READ);
            fs["V_cv"] >> V_cv;
        }
        catch (...)
        {
        };

        if (!fs.isOpened())
        {
            COUT << "Failed" << std::endl;
            break;
        }

        fs.release();

        /* calculate camera position */
        C_cv = PLANE * V_cv.inv();

        /* notify it */
//        COUT << "C_cv[" << img_idx <<"]=" << C_cv << std::endl;

        // create vrpn data filename
        snprintf(path, sizeof(path), "%s\\detect_aruco_markers_%d_vrpn_%03d.yaml",
            img_path, img_ses, img_idx);

        COUT << path << "... ";

        // read vrpn result
        try
        {
            fs.open(path, cv::FileStorage::READ);

            // cam
            fs["vrpn_cam_xyz"] >> VRPN_xyz;
            fs["vrpn_cam_quat"] >> VRPN_quat;
            conv1(VRPN_xyz, VRPN_quat, &vrpn_cam);

            // stk
            fs["vrpn_stk_xyz"] >> VRPN_xyz;
            fs["vrpn_stk_quat"] >> VRPN_quat;
            conv1(VRPN_xyz, VRPN_quat, &pos);
            wt_openvr_to_ue(&pos, &vrpn_stk);

            // stk
            fs["vrpn_mnt_xyz"] >> VRPN_xyz;
            fs["vrpn_mnt_quat"] >> VRPN_quat;
            conv1(VRPN_xyz, VRPN_quat, &pos);
            wt_openvr_to_ue(&pos, &vrpn_mnt);
        }
        catch (...)
        {
        };

        if (!fs.isOpened())
        {
            COUT << "Failed" << std::endl;
            break;
        }

        fs.release();

        COUT << " OK" << std::endl;

        /* convert CV to UE axis */
        wt_Mat4_to_quat_xyz(C_cv, &cam_cv);
        wt_cv_to_ue(&cam_cv, &cam_ue);
        print_xyz_quat("  cam_ue", &cam_ue);

        /* print some results */
        print_xyz_quat("vrpn_cam", &vrpn_cam);
        print_xyz_quat("vrpn_mnt", &vrpn_mnt);
        print_xyz_quat("vrpn_stk", &vrpn_stk);

        /* save */
        CAMs.push_back(cam_ue);
        STKs.push_back(vrpn_stk);
        MNTs.push_back(vrpn_mnt);

        img_idx++;
    }

    if (!CAMs.size())
    {
        COUT << "No source datas" << std::endl;
        exit(0);
    };

    COUT << "--------------------------------------------------" << std::endl;

    if(0)
    {
        char path[1024];
        COUT << "Searching data using cv::calibrateRobotWorldHandEye" << std::endl;

        q_xyz_quat_type arm, ref, base2world, gripper2cam, cam2gripper;
        std::vector<q_xyz_quat_type> trks, cams;

        q_vec_copy(arm.xyz, vec0);
        q_copy(arm.quat, prerot0);

        q_vec_copy(ref.xyz, vec0);
        q_copy(ref.quat, prerot0);

        calc_tracking_model(&arm, &ref, trks, cams);

        std::vector<cv::Mat> R_world2cam, t_world2cam, R_base2gripper, t_base2gripper;
        std::vector<cv::Mat> R_cam2world, t_cam2world, R_gripper2base, t_gripper2base;
        cv::Mat R_base2world, t_base2world, R_gripper2cam, t_gripper2cam;
        cv::Mat R_cam2gripper, t_cam2gripper;

        for (i = 0; i < trks.size(); i += 10)
        {
            cv::Mat V, rvec, tvec;

            wt_quat_xyz_to_Mat4(&CAMs[i], V);
            wt_Mat4_to_rvec_tvec(V, rvec, tvec);
            R_world2cam.push_back(rvec);
            t_world2cam.push_back(tvec);
            V = V.inv();
            wt_Mat4_to_rvec_tvec(V, rvec, tvec);
            R_cam2world.push_back(rvec);
            t_cam2world.push_back(tvec);

            wt_quat_xyz_to_Mat4(&cams[i], V);
            wt_Mat4_to_rvec_tvec(V, rvec, tvec);
            R_base2gripper.push_back(rvec);
            t_base2gripper.push_back(tvec);
            V = V.inv();
            wt_Mat4_to_rvec_tvec(V, rvec, tvec);
            R_gripper2base.push_back(rvec);
            t_gripper2base.push_back(tvec);
        }

        snprintf(path, sizeof(path), "%s\\detect_aruco_markers_%d_hand_eye.yaml",
            img_path, img_ses);

        cv::FileStorage fs(path, cv::FileStorage::WRITE);
        fs << "R_world2cam" << R_world2cam;
        fs << "t_world2cam" << t_world2cam;
        fs << "R_base2gripper" << R_base2gripper;
        fs << "t_base2gripper" << t_base2gripper;
        fs.release();

        static const int calibrateRobotWorldHandEye_method_ids[] = { cv::CALIB_ROBOT_WORLD_HAND_EYE_LI , cv::CALIB_ROBOT_WORLD_HAND_EYE_SHAH, 0 };
        static const char* calibrateRobotWorldHandEye_method_names[] = { "CALIB_ROBOT_WORLD_HAND_EYE_LI" , "CALIB_ROBOT_WORLD_HAND_EYE_SHAH", NULL};
        for (i = 0; calibrateRobotWorldHandEye_method_names[i]; i++)
        {
            COUT << calibrateRobotWorldHandEye_method_names[i] << std::endl;
            cv::calibrateRobotWorldHandEye(
                R_world2cam, t_world2cam, R_base2gripper, t_base2gripper,
                R_base2world, t_base2world, R_gripper2cam, t_gripper2cam,
                (cv::RobotWorldHandEyeCalibrationMethod)calibrateRobotWorldHandEye_method_ids[i]);
            wt_rvec_tvec_to_quat_xyz(R_base2world, t_base2world, &base2world);
            wt_rvec_tvec_to_quat_xyz(R_gripper2cam, t_gripper2cam, &gripper2cam);
            print_xyz_quat("base2world", &base2world);
            print_xyz_quat("gripper2cam", &gripper2cam);
        };

        static const int calibrateHandEye_method_ids[] = { cv::CALIB_HAND_EYE_TSAI, cv::CALIB_HAND_EYE_PARK, cv::CALIB_HAND_EYE_HORAUD, cv::CALIB_HAND_EYE_ANDREFF, cv::CALIB_HAND_EYE_DANIILIDIS, 0 };
        static const char* calibrateHandEye_method_names[] = { "CALIB_HAND_EYE_TSAI", "CALIB_HAND_EYE_PARK", "CALIB_HAND_EYE_HORAUD", "CALIB_HAND_EYE_ANDREFF", "CALIB_HAND_EYE_DANIILIDIS", NULL};
        for (i = 0; calibrateHandEye_method_names[i]; i++)
        {
            COUT << calibrateHandEye_method_names[i] << std::endl;
            cv::calibrateHandEye(
                R_gripper2base, t_gripper2base, R_world2cam, t_world2cam,
                R_cam2gripper, t_cam2gripper,
                (cv::HandEyeCalibrationMethod)calibrateHandEye_method_ids[i]);
            wt_rvec_tvec_to_quat_xyz(R_cam2gripper, t_cam2gripper, &cam2gripper);
            print_xyz_quat("cam2gripper", &cam2gripper);
        };
    };


    {
        q_xyz_quat_type arm, ref;

        q_vec_copy(arm.xyz, vec0);
        q_copy(arm.quat, prerot0);

        q_vec_copy(ref.xyz, vec0);
        q_copy(ref.quat, prerot0);

        COUT << "--------------------------------------------------" << std::endl;
        COUT << "Searching for ARM rotation" << std::endl;
        task_find_arm_rot(&arm, &ref);
        print_xyz_quat("REF", &ref);
        print_xyz_quat("ARM", &arm);

        COUT << "--------------------------------------------------" << std::endl;
        COUT << "Searching for ARM/REF position" << std::endl;
        task_find_pos2(&arm, &ref, task_find_pos2_criteria);
        print_xyz_quat("REF", &ref);
        print_xyz_quat("ARM", &arm);
#if 0
        COUT << "--------------------------------------------------" << std::endl;
        COUT << "Searching for ARM rotation(2)" << std::endl;
        task_find_arm_rot(&arm, &ref);
        print_xyz_quat("REF", &ref);
        print_xyz_quat("ARM", &arm);

        COUT << "--------------------------------------------------" << std::endl;
        COUT << "Searching for ARM/REF position(2)" << std::endl;
        task_find_pos2(&arm, &ref, task_find_pos2_criteria);
        print_xyz_quat("REF", &ref);
        print_xyz_quat("ARM", &arm);
#endif
        COUT << "--------------------------------------------------" << std::endl;
        COUT << "Distance analisys:" << std::endl;
        std::vector<q_xyz_quat_type> diffs;
        calc_tracking_model_diff(&arm, &ref, diffs, true);

    };

    COUT << "--------------------------------------------------" << std::endl;


    return 0;
}
