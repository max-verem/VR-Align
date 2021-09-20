#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <windows.h>
#include <time.h>
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

/// https://learnopencv.com/rotation-matrix-to-euler-angles/


#ifdef _DEBUG
#pragma comment(lib, "opencv_world453d.lib")
#else
#pragma comment(lib, "opencv_world453.lib")
#endif

static void zero_matrix(q_matrix_type m)
{
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) m[i][j] = 0.0;
}

int main(int argc, char** argv)
{
    cv::Size imagesize;
    std::vector<std::string> imageNames, pngNames;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;

    int i, j, a;
    char
        src_path[4096] = "D:\\projects\\VR-Align\\_2021-06-21.chess",
        dst_path[4096] = "",
        ext[128] = ".png",
        tmp[4096];
    double
        dpi = 400,
        chess_board_step = 160,
        chess_board_dim[2] = { 24, 11 },
        world_window_pos[3] = { -10.24 / 2.0, -(2.168 + 5.28 / 2), 3.0 },
        chess_board_offset[2] = { 80.0, 80.0 };

#define ARG_CHECK_BEGIN(PARAM_NAME, PARAMS_CNT) if (!strcmp("--" PARAM_NAME, argv[i]) && (PARAMS_CNT + i) < argc) {
#define ARG_CHECK_END(PARAMS_CNT) i += PARAMS_CNT + 1; }

    for (i = 1; i < argc;)
    {
        ARG_CHECK_BEGIN("world-window-pos", 3)
            world_window_pos[0] = atof(argv[i + 1]);
            world_window_pos[1] = atof(argv[i + 2]);
            world_window_pos[2] = atof(argv[i + 3]);
        ARG_CHECK_END(3)
        else
        ARG_CHECK_BEGIN("chess-board-offset", 2)
            chess_board_offset[0] = atof(argv[i + 1]);
            chess_board_offset[1] = atof(argv[i + 2]);
        ARG_CHECK_END(2)
        else
        ARG_CHECK_BEGIN("chess-board-dim", 2)
            chess_board_dim[0] = atof(argv[i + 1]);
            chess_board_dim[1] = atof(argv[i + 2]);
        ARG_CHECK_END(2)
        else
        ARG_CHECK_BEGIN("chess-board-step", 1)
            chess_board_step = atof(argv[i + 1]);
        ARG_CHECK_END(1)
        else
        ARG_CHECK_BEGIN("dpi", 1)
            dpi = atof(argv[i + 1]);
        ARG_CHECK_END(1)
        else
        ARG_CHECK_BEGIN("src-path", 1)
            strcpy(src_path, argv[i + 1]);
        ARG_CHECK_END(1)
        else
        ARG_CHECK_BEGIN("dst-path", 1)
            strcpy(dst_path, argv[i + 1]);
        ARG_CHECK_END(1)
        else
        ARG_CHECK_BEGIN("src-ext", 1)
            snprintf(ext, sizeof(ext), ".%s", argv[i + 1]);
        ARG_CHECK_END(1)
        else
        {
            fprintf(stderr, "Error! Failed to parse arg #%d [%s]\n", i, argv[i]);
            exit(1);
        };
    };

    cv::Size patternsize(chess_board_dim[0] - 1, chess_board_dim[1] - 1); //interior number of corners
    cv::Point3f top_left_world(world_window_pos[0], world_window_pos[1], 0);

    // build corners coordinates
    printf("inputObjectPoints=[\n");
    std::vector<cv::Point3f> inputObjectPoints;
    for (j = 0; j < patternsize.height; j++)
    {
        printf("    ");
        for (i = 0; i < patternsize.width; i++)
        {
            cv::Point3f ref
            (
                top_left_world.x + (chess_board_offset[0] + chess_board_step + chess_board_step * i) / dpi,
                top_left_world.y + (chess_board_offset[1] + chess_board_step + chess_board_step * j) / dpi,
                0.0
            );

            inputObjectPoints.push_back(ref);

            printf(" {%.4f, %.4f },", ref.x, ref.y);
        };
        printf("\n");
    };
    printf("]\n");

    // find files in PNFs directory
    WIN32_FIND_DATA ffd;
    snprintf(tmp, sizeof(tmp), "%s\\*%s", src_path, ext);
    HANDLE hFind = FindFirstFile(tmp, &ffd);
    if (hFind == INVALID_HANDLE_VALUE)
    {
        fprintf(stderr, "Error! FindFirstFile(%s) failed (%d)\n", src_path, GetLastError());
        exit(1);
    };
    // iterate
    do
    {
        // skip directory
        if (!(ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
        {
            // check if ext present
            char *fext = strstr(ffd.cFileName, ext);
            if (fext)
            {
                // check if finished with .png
                if (!strcmp(fext, ext))
                    pngNames.push_back(ffd.cFileName);
            }
        }
    }
    while (FindNextFile(hFind, &ffd) != 0);

    // close find handle
    FindClose(hFind);

    // check if any images found
    if (!pngNames.size())
    {
        fprintf(stderr, "Error! No files (*%s) found in [%s]\n", ext, src_path);
        exit(1);
    };

    for (a = 0; a < pngNames.size(); a++)
    {
        cv::Mat frame, gray;
        std::vector<cv::Point2f> corners;

        snprintf(tmp, sizeof(tmp), "%s\\%s", src_path, pngNames[a].c_str());
        frame = cv::imread(tmp);
        if (!frame.cols || !frame.rows)
        {
            fprintf(stderr, "Failed to read [%s]\n", pngNames[a].c_str());
            continue;
        }

        printf("File [%s] read: width=%d, height=%d\n", pngNames[a].c_str(), frame.cols, frame.rows);

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        printf("Looking for corners if [%s] ...\n", pngNames[a].c_str());
        bool patternfound = cv::findChessboardCorners(gray, patternsize, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
        printf("... finished, patternfound=%d\n", patternfound);

        if (!patternfound)
        {
            printf("pattern not found\n");
            continue;
        }

        // save image
        imageNames.push_back(pngNames[a]);

        // more precise
        printf("Precising corner...\n");
        cv::find4QuadCornerSubpix(gray, corners, patternsize);
        printf("... finished\n");

        // Since we only have one view, set the point sets as the first element of the vectors
        imagePoints.push_back(corners);
        objectPoints.push_back(inputObjectPoints);

        imagesize = gray.size();
        printf("imagesize: width=%d, height=%d\n", imagesize.width, imagesize.height);
    };

    if (!objectPoints.size())
    {
        fprintf(stderr, "Error! Nothing found\n");
        exit(1);
    };

    // magic ?
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

    printf("Calibrating...\n");
    double r = cv::calibrateCamera(objectPoints, imagePoints, imagesize, cameraMatrix, distCoeffs, rvecs, tvecs);
    printf("... finished, r=%f\n", r);

    // create file
    snprintf(tmp, sizeof(tmp), "%s\\ParseChessBoard-%lld.yaml", src_path, time(NULL));
    cv::FileStorage fs(tmp, cv::FileStorage::WRITE);

    // save all datas
    fs << "count" << (int)objectPoints.size();
    fs << "cameraMatrix" << cameraMatrix;
    fs << "distCoeffs" << distCoeffs;
    fs << "rvecs" << rvecs;
    fs << "tvecs" << tvecs;
    fs << "imageNames" << imageNames;
    fs << "depth" << world_window_pos[2];
    fs.release();

    // exit if not dst path specified
    if (!dst_path[0])
        exit(0);

    // save updated images
    printf("Drawing images (to %s)...\n", dst_path);
    for (a = 0; a < tvecs.size(); a++)
    {
        cv::Mat frame;
        std::vector<cv::Point2f> new_corners;
        cv::projectPoints(objectPoints[a], rvecs[a], tvecs[a], cameraMatrix, distCoeffs, new_corners);
        snprintf(tmp, sizeof(tmp), "%s\\%s", src_path, imageNames[a].c_str());
        frame = cv::imread(tmp);
        cv::drawChessboardCorners(frame, patternsize, new_corners, true);
        cv::drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs[a], tvecs[a], 4, 2);

        // own axis
        {
            std::vector<cv::Point3f> new_axis;

            new_axis.push_back(cv::Point3f(0.0, 0.0, 0.0) + top_left_world);
            new_axis.push_back(cv::Point3f(1.0, 0.0, 0.0) + top_left_world);
            new_axis.push_back(cv::Point3f(0.0, 1.0, 0.0) + top_left_world);
            new_axis.push_back(cv::Point3f(0.0, 0.0, 1.0) + top_left_world);
            new_axis.push_back(cv::Point3f(0.0, 0.0, -1.0) + top_left_world);

            new_corners.clear();

            cv::projectPoints(new_axis, rvecs[a], tvecs[a], cameraMatrix, distCoeffs, new_corners);

            cv::arrowedLine(frame, new_corners[0], new_corners[1], cv::Scalar(220, 220, 0), 4);
            cv::arrowedLine(frame, new_corners[0], new_corners[2], cv::Scalar(0, 220, 220), 4);
            cv::arrowedLine(frame, new_corners[0], new_corners[3], cv::Scalar(220, 0, 220), 4);
            cv::arrowedLine(frame, new_corners[0], new_corners[4], cv::Scalar(220, 220, 220), 4);
        };

        snprintf(tmp, sizeof(tmp), "%s\\%s", dst_path, imageNames[a].c_str());
        cv::imwrite(tmp, frame);
    };
    printf("... saved, %zd images\n", tvecs.size());

    return 0;
}

