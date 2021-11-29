/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.

                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

Copyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

#define _CRT_SECURE_NO_WARNINGS

#include <opencv2/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <iostream>
#include <time.h>

#ifdef _DEBUG
#pragma comment(lib, "opencv_world454d.lib")
#else
#pragma comment(lib, "opencv_world454.lib")
#endif

using namespace std;
using namespace cv;


namespace {
const char* about = "Pose estimation using a ChArUco board";
const char* keys  =
        "{vm       |       | mode to select for cam, 1920x1080@25 for example}"
        "{startA   |       | Starting number for ids used for A board }"
        "{startB   |       | Starting number for ids used for B board }"
        "{dpiA     |       | DPI for ids used for A board }"
        "{dpiB     |       | DPI used for B board }"
        "{wA       |       | Number of squares in X direction }"
        "{hA       |       | Number of squares in Y direction }"
        "{slA      |       | Square side length (in meters) }"
        "{mlA      |       | Marker side length (in meters) }"
        "{dA       |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"

        "{wB       |       | Number of squares in X direction }"
        "{hB       |       | Number of squares in Y direction }"
        "{slB      |       | Square side length (in meters) }"
        "{mlB      |       | Marker side length (in meters) }"
        "{dB       |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"

        "{c        |       | Output file with calibrated camera parameters }"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{dp       |       | File of marker detector parameters }"
        "{rs       |       | Apply refind strategy }"
        "{r        |       | show rejected candidates too }";
}

/**
 */
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}


/**
 */
static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}


#include <quat.h>
#pragma comment(lib, "quatlib.lib")

#include "../common/wt.h"
#include "../common/wt_cv.h"
#include "../common/print_vecs.h"

/**
 */
int main(int argc, char *argv[]) {
    int b;
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(argc < 6) {
        parser.printMessage();
        return 0;
    }

    int squaresX[2] = { parser.get<int>("wA"), parser.get<int>("wB") };
    int squaresY[2] = { parser.get<int>("hA"), parser.get<int>("hB") };
    float dpi[2] = { parser.has("dpiA") ? parser.get<float>("dpiA") : 1.0, parser.has("dpiB") ? parser.get<float>("dpiB") : 1.0 };
    float squareLength[2] = { parser.get<float>("slA") / dpi[0], parser.get<float>("slB") / dpi[1] };
    float markerLength[2] = { parser.get<float>("mlA") / dpi[0], parser.get<float>("mlB") / dpi[1] };
    int dictionaryId[2] = { parser.get<int>("dA"), parser.get<int>("dB") };

    bool showRejected = parser.has("r");
    bool refindStrategy = parser.has("rs");
    int camId = parser.get<int>("ci");

    String video;
    if(parser.has("v")) {
        video = parser.get<String>("v");
    }

    Mat camMatrix, distCoeffs;
    if(parser.has("c")) {
        bool readOk = readCameraParameters(parser.get<string>("c"), camMatrix, distCoeffs);
        if(!readOk) {
            cerr << "Invalid camera file" << endl;
            return 0;
        }
    }


    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    if(parser.has("dp")) {
        bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
        if(!readOk) {
            cerr << "Invalid detector parameters file" << endl;
            return 0;
        }
    }

    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }

    Ptr<aruco::Dictionary> dictionary[2] =
    {
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId[0])),
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId[1]))
    };

    VideoCapture inputVideo;
    int waitTime;
    if(!video.empty()) {
        inputVideo.open(video);
        waitTime = 0;
    } else {
        inputVideo.open(camId);
        if (parser.has("vm"))
        {
            int w, h, f;

            if (3 == sscanf(parser.get<string>("vm").c_str(), "%dx%d@%d", &w, &h, &f))
            {
                inputVideo.set(CAP_PROP_FRAME_WIDTH, w);
                inputVideo.set(CAP_PROP_FRAME_HEIGHT, h);
                inputVideo.set(CAP_PROP_FPS, f);
            }
            else
                cerr << "Failed to parse [" << parser.get<string>("vm") << "]" << endl;
        };

        std::cout << "getBackendName=" << inputVideo.getBackendName() << endl;
        std::cout << "CAP_PROP_FRAME_WIDTH=" << inputVideo.get(CAP_PROP_FRAME_WIDTH) << ", CAP_PROP_FRAME_HEIGHT=" << inputVideo.get(CAP_PROP_FRAME_HEIGHT) << ", CAP_PROP_FPS=" << inputVideo.get(CAP_PROP_FPS) << endl;
        waitTime = 10;
    }

    float axisLength[2];
    for(b =0; b < 2; b++)
        axisLength[b] = 0.5f * ((float)min(squaresX[b], squaresY[b]) * (squareLength[b]));

    // create charuco board object
    Ptr<aruco::CharucoBoard> charucoboard[2] =
    {
        aruco::CharucoBoard::create(squaresX[0], squaresY[0], squareLength[0], markerLength[0], dictionary[0]),
        aruco::CharucoBoard::create(squaresX[1], squaresY[1], squareLength[1], markerLength[1], dictionary[1])
    };

    int start[2] = { 0 , 0 };

    if(parser.has("startA"))
        start[0] = parser.get<int>("startA");
    if (parser.has("startB"))
        start[1] = parser.get<int>("startB");

    for (b = 0; b < 2; b++)
    {

        for (int i = 0; i < charucoboard[b]->ids.size(); i++)
            charucoboard[b]->ids[i] += start[b];
    }

    Ptr<aruco::Board> board[2] =
    {
        charucoboard[0].staticCast<aruco::Board>(),
        charucoboard[1].staticCast<aruco::Board>()
    };

    double totalTime = 0;
    int totalIterations = 0;
    int save_results = 0;
    time_t start_time = time(NULL);
    int saved_images = 0;

    while(inputVideo.grab())
    {
        cv::Mat V_baords[2];
        Mat image, imageCopy;
        inputVideo.retrieve(image);

//        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

        double tick = (double)getTickCount();
        bool validPose[2];
        vector< int > markerIds[2], charucoIds[2];
        vector< vector< Point2f > > markerCorners[2], rejectedMarkers[2];
        vector< Point2f > charucoCorners[2];
        Vec3d rvec[2], tvec[2];
        int interpolatedCorners[2];

        for (b = 0; b < 2; b++)
        {
            // detect markers
            aruco::detectMarkers(image, dictionary[b], markerCorners[b], markerIds[b], detectorParams,
                rejectedMarkers[b]);

            // refind strategy to detect more markers
            if (refindStrategy)
                aruco::refineDetectedMarkers(image, board[b], markerCorners[b], markerIds[b], rejectedMarkers[b],
                    camMatrix, distCoeffs);

            // interpolate charuco corners
            interpolatedCorners[b] = 0;
            if (markerIds[b].size() > 0)
                interpolatedCorners[b] =
                aruco::interpolateCornersCharuco(markerCorners[b], markerIds[b], image, charucoboard[b],
                    charucoCorners[b], charucoIds[b], camMatrix, distCoeffs);

            // estimate charuco board pose
            validPose[b] = false;
            if (camMatrix.total() != 0)
                validPose[b] = aruco::estimatePoseCharucoBoard(charucoCorners[b],
                    charucoIds[b], charucoboard[b],
                    camMatrix, distCoeffs, rvec[b], tvec[b]);

        }


        double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
        totalTime += currentTime;
        totalIterations++;
        if(totalIterations % 30 == 0) {
            cout << "Detection Time = " << currentTime * 1000 << " ms "
                 << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
        }

        // draw results
        image.copyTo(imageCopy);

        for (b = 0; b < 2; b++)
        {

            if (markerIds[b].size() > 0) {
                aruco::drawDetectedMarkers(imageCopy, markerCorners[b]);
            }

            if (showRejected && rejectedMarkers[b].size() > 0)
                aruco::drawDetectedMarkers(imageCopy, rejectedMarkers[b], noArray(), Scalar(100, 0, 255));

            if (interpolatedCorners[b] > 0) {
                Scalar color;
                color = Scalar(255, 0, 0);
                aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners[b], charucoIds[b], color);
            }

            if (validPose[b])
            {
                aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvec[b], tvec[b], axisLength[b]);

                // some previous code
                // at https://gist.github.com/max-verem/eda6a029c9cfd53d60739b8f7d5724ad

                cv::Mat V_charuco;

                // convert rvec/tvec to matrix
                wt_rvec_tvec_to_Mat4(rvec[b], tvec[b], V_charuco);

                // define rotation matrix for Charuco-to-CV
                cv::Mat ROT_X = (cv::Mat_<double>(4, 4) <<
                    1.0, 0.0, 0.0, 0.0,
                    0.0, -1.0, 0.0, 0.0,
                    0.0, 0.0, -1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0);

                // find translation in OpenCV axis
                cv::Mat V_cv = V_charuco * ROT_X;

                // translate matrix to center
                cv::Mat TRANS_CENTR = (cv::Mat_<double>(4, 4) <<
                    1.0, 0.0, 0.0, squaresX[b] * squareLength[b] / 2.0,
                    0.0, 1.0, 0.0, -squaresY[b] * squareLength[b] / 2.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0);

                V_cv = V_cv * TRANS_CENTR;

                V_baords[b] = V_cv;

                // own axis
                {
                    std::vector<cv::Point3f> new_axis;
                    std::vector<cv::Point2f> new_corners;
                    double axis_length = 2 * squareLength[b];

                    new_axis.push_back(cv::Point3f(0.0, 0.0, 0.0));
                    new_axis.push_back(cv::Point3f(axis_length, 0.0, 0.0));
                    new_axis.push_back(cv::Point3f(0.0, axis_length, 0.0));
                    new_axis.push_back(cv::Point3f(0.0, 0.0, axis_length));
                    new_axis.push_back(cv::Point3f(0.0, 0.0, -axis_length));

                    cv::Mat rvec_cv, tvec_cv;
                    wt_Mat4_to_rvec_tvec(V_cv, rvec_cv, tvec_cv);

                    cv::projectPoints(new_axis, rvec_cv, tvec_cv, camMatrix, distCoeffs, new_corners);

                    // color is cv::Scalar(B, G, R)
                    cv::arrowedLine(imageCopy, new_corners[0], new_corners[1], cv::Scalar(0, 0, 220), 4);   // X
                    cv::arrowedLine(imageCopy, new_corners[0], new_corners[2], cv::Scalar(0, 220, 0), 4);   // Y
                    cv::arrowedLine(imageCopy, new_corners[0], new_corners[3], cv::Scalar(220, 0, 0), 4);   // Z
                    cv::arrowedLine(imageCopy, new_corners[0], new_corners[4], cv::Scalar(0, 220, 220), 4); // -Z
                };
            }

        }

        putText(imageCopy, "Press 's' to save current detection. 'ESC' to finish",
            Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);

        imshow("out", imageCopy);
        char key = (char)waitKey(waitTime);
        if(key == 27) break;
        if (key == 's')
            save_results++;

        if (save_results && validPose[0] && validPose[1])
        {
            char tmp[1024];

            save_results = 0;

            // create file
            snprintf(tmp, sizeof(tmp), "detect_two_board_charuco_%lld_%03d.yaml", start_time, saved_images);
            cv::FileStorage fs(tmp, cv::FileStorage::WRITE);

            // save all datas
            fs << "A" << V_baords[0];
            fs << "B" << V_baords[1];
            fs.release();

            if (video.empty()) {
                snprintf(tmp, sizeof(tmp), "detect_two_board_charuco_%lld_origin_%03d.png", start_time, saved_images);
                imwrite(tmp, image);
            }

            snprintf(tmp, sizeof(tmp), "detect_two_board_charuco_%lld_detect_%03d.png", start_time, saved_images);
            imwrite(tmp, imageCopy);

            saved_images++;

            printf("Saving results!\n");
        }
    }

    return 0;
}
