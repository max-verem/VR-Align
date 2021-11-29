
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

#include <quat.h>
#pragma comment(lib, "quatlib.lib")

#include "../common/wt.h"
#include "../common/wt_cv.h"
#include "../common/print_vecs.h"
#include "../common/avg_xyz_quat.h"

int main(int argc, char *argv[])
{
    int i;
    q_xyz_quat_type d, pos;
    cv::Mat pA;

#if 1
    pA = (cv::Mat_<double>(4, 4) << // Rx(-90)
/*
        | 1     0       0       |
        | 0     cosA    -sinA   |
        | 0     sinA    cosA    |
*/

        1.0,  0.0,  0.0, 0.0,
        0.0,  0.0,  1.0, 0.0,
        0.0, -1.0,  0.0, 3.0,
        0.0,  0.0,  0.0, 1.0);

    std::cout << "pA=" << pA << std::endl;
    wt_Mat4_to_quat_xyz(pA, &pos);
    print_xyz_quat("pos", &pos);
    std::cout << std::endl;
#else
    q_from_euler(pos.quat,
        Q_DEG_TO_RAD(  0.0),
        Q_DEG_TO_RAD(  0.0),
        Q_DEG_TO_RAD(-90.0)); // about X for -90

    pos.xyz[0] = 0.0;
    pos.xyz[1] = 0.0;
    pos.xyz[2] = 3.0;

    wt_quat_xyz_to_Mat4(&pos, pA);
    std::cout << "pA=" << pA << std::endl;
    print_xyz_quat("pos", &pos);
#endif

#if 0
    for (i = 1; i < argc; i++)
    {
        cv::Mat cA, cB, D;

        // read parse result
        cv::FileStorage fs(argv[i], cv::FileStorage::READ);
        fs["A"] >> cA;
        fs["B"] >> cB;
        fs.release();

        D = cB.inv() * cA;
        wt_Mat4_to_quat_xyz(D, &d);
        print_xyz_quat("B^-1 * A", &d);
    }

    printf("\n");

    for (i = 1; i < argc; i++)
    {
        cv::Mat cA, cB, D;

        // read parse result
        cv::FileStorage fs(argv[i], cv::FileStorage::READ);
        fs["A"] >> cA;
        fs["B"] >> cB;
        fs.release();

        D = cA.inv() * cB;
        wt_Mat4_to_quat_xyz(D, &d);
        print_xyz_quat("A^-1 * B", &d);
    }
#endif

    std::vector<q_xyz_quat_type> pBs;

    for (i = 1; i < argc; i++)
    {
        cv::Mat cA, cB, pB;

        // read parse result
        cv::FileStorage fs(argv[i], cv::FileStorage::READ);
        fs["A"] >> cA;
        fs["B"] >> cB;
        fs.release();

        pB = pA * cA.inv() * cB;

        wt_Mat4_to_quat_xyz(pB, &d);
        print_xyz_quat("pB", &d);
        pBs.push_back(d);
    }
    std::cout << std::endl;

    q_xyz_quat_type pBavgCV, pBavgUE;
    avg_xyz_quat_v1(pBs, &pBavgCV);
    print_xyz_quat("pBavg(CV)", &pBavgCV);

    wt_cv_to_ue(&pBavgCV, &pBavgUE);
    print_xyz_quat("pBavg(UE)", &pBavgUE);

    return 0;
}
