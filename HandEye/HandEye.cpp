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

static void pose_to_list(const q_xyz_quat_type *pose, std::vector<cv::Mat> &R_, std::vector<cv::Mat> &t_, bool inv = false)
{
    cv::Mat V, R, t;
    q_matrix_type m;
    q_xyz_quat_type view;

    if (inv)
        q_xyz_quat_invert(&view, pose);
    else
        view = *pose;

    q_xyz_quat_to_row_matrix(m, &view);
///    print_matrix("m=", m);

    V = cv::Mat::eye(4, 4, CV_64F);
    wt_row_matrix_to_Mat4(m, V);
///    std::cout << "V=" << V << "\n";

    R = V(cv::Range(0, 3), cv::Range(0, 3));
///    std::cout << "R=" << R << "\n";
    t = V(cv::Range(0, 3), cv::Range(3, 4));
///    std::cout << "t=" << t << "\n";

    R_.push_back(R);
    t_.push_back(t);
}

int main(int argc, char** argv)
{
    int a, i, j;

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
        parse_chess_board_yaml[4096] = "D:\\projects\\VR-Align\\_2021-06-21.chess\\ParseChessBoard-1628833438.yaml";

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
        {
            fprintf(stderr, "Error! Failed to parse arg #%d [%s]\n", i, argv[i]);
            exit(1);
        };
    };

    std::vector<q_xyz_quat_type> trks, cams;

    // load sources
    chess_results_load(parse_chess_board_yaml);

    print_xyz_quat("REF", &ref);
    print_xyz_quat("ARM", &arm);

    // check which task specified
    if (!task)
    {
        fprintf(stderr, "Error, please specify --task=<calibrateRobotWorldHandEye|calibrateHandEye>\n");
        exit(1);
    }
    else if (!strcmp("calibrateRobotWorldHandEye", task))
    {
        static const cv::RobotWorldHandEyeCalibrationMethod method_ids[] = { cv::CALIB_ROBOT_WORLD_HAND_EYE_SHAH, cv::CALIB_ROBOT_WORLD_HAND_EYE_LI };
        static const char* method_names[] = { "CALIB_ROBOT_WORLD_HAND_EYE_SHAH", "CALIB_ROBOT_WORLD_HAND_EYE_LI", NULL };

        trks.clear();
        cams.clear();
        calc_tracking_model_from_chess_results(&arm, &ref, trks, cams);

        for(int m = 0; method_names[m]; m++){
        for (int dir = 0; dir < 4; dir++)
        {
            bool i1 = dir & 1, i2 = dir & 2;
            std::vector<cv::Mat> R_world2cam, t_world2cam, R_base2gripper, t_base2gripper;

            for (a = 0; a < chess_results.size(); a++)
            {
                pose_to_list(&cams[a], R_base2gripper, t_base2gripper, i1);
                pose_to_list(&chess_results[a]->pose, R_world2cam, t_world2cam, i2);
            };

            cv::Mat R_base2world, t_base2world, R_gripper2cam, t_gripper2cam;

            cv::calibrateRobotWorldHandEye(R_world2cam, t_world2cam, R_base2gripper, t_base2gripper,
                R_base2world, t_base2world, R_gripper2cam, t_gripper2cam, method_ids[m]);

            printf("cam inv=%d, cv inv=%d, method=%s\n", i1, i2, method_names[m]);

            q_xyz_quat_type tmp;
            q_xyz_quat_type tmp_i;

            wt_rvec_tvec_to_quat_xyz(R_base2world, t_base2world, &tmp);
            print_xyz_quat("  base2world", &tmp);
            q_xyz_quat_invert(&tmp_i, &tmp);
            print_xyz_quat(" base2world'", &tmp_i);

            wt_rvec_tvec_to_quat_xyz(R_gripper2cam, t_gripper2cam, &tmp);
            print_xyz_quat(" gripper2cam", &tmp);
            q_xyz_quat_invert(&tmp_i, &tmp);
            print_xyz_quat("gripper2cam'", &tmp_i);
            
            printf("\n");
        }printf("----\n\n");}

        printf("Done!\n");
    }
    else if (!strcmp("calibrateHandEye", task))
    {
        static const cv::HandEyeCalibrationMethod method_ids[] = { cv::CALIB_HAND_EYE_TSAI, cv::CALIB_HAND_EYE_PARK, cv::CALIB_HAND_EYE_HORAUD, cv::CALIB_HAND_EYE_ANDREFF, cv::CALIB_HAND_EYE_DANIILIDIS };
        static const char* method_names[] = { "CALIB_HAND_EYE_TSAI", "CALIB_HAND_EYE_PARK", "CALIB_HAND_EYE_HORAUD", "CALIB_HAND_EYE_ANDREFF", "CALIB_HAND_EYE_DANIILIDIS", NULL };

        q_xyz_quat_type ref;
        q_from_euler(ref.quat, Q_DEG_TO_RAD(0), Q_DEG_TO_RAD(0), Q_DEG_TO_RAD(0));
        q_vec_set(ref.xyz, 0.0, 0.0, 0.0);

        for (int m = 0; method_names[m]; m++) {
            for (int dir = 0; dir < 4; dir++)
            {
                bool i1 = dir & 1, i2 = dir & 2;
                std::vector<cv::Mat> R_gripper2base, R_target2cam, t_gripper2base, t_target2cam;

                for (a = 0; a < chess_results.size(); a++)
                {
                    cv::Mat mat;
                    q_xyz_quat_type cam, stk, mnt;

                    // static tracker
#if 0
                    stk = chess_results[a]->vrpns[3].pose;
#else
                    wt_rot_mask(&chess_results[i]->vrpns[3].pose,
                        1.0, 0.0, 0.0, &stk);
#endif
                    // mounter tracker
                    mnt = chess_results[a]->vrpns[2].pose;

                    // calc tracker
                    wt_A_to_B
                    (
                        &stk,    // static tracker
                        &ref,
                        &mnt,    // mounted tracker
                        &cam
                    );

                    pose_to_list(&cam, R_gripper2base, t_gripper2base, i1);
                    pose_to_list(&chess_results[a]->pose, R_target2cam, t_target2cam, i2);
                };

                cv::Mat R_cam2gripper, t_cam2gripper;
                cv::calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper, t_cam2gripper, method_ids[m]);

                q_xyz_quat_type tmp;
                wt_rvec_tvec_to_quat_xyz(R_cam2gripper, t_cam2gripper, &tmp);

                printf("cam inv=%d, cv inv=%d, method=%s\n", i1, i2, method_names[m]);
                print_xyz_quat("cam2gripper=", &tmp);
                q_xyz_quat_type tmp_i;
                q_xyz_quat_invert(&tmp_i, &tmp);
                print_xyz_quat("cam2gripper'=", &tmp_i);
            }printf("\n");
        }

        printf("Done!\n");
    }
    else
    {
        fprintf(stderr, "Error! Failed to recognize task [%s]\n", task);
        exit(0);
    };

    return 0;
};
