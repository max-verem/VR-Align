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
        "{img-path |       | Path for input images sequences stored, files named like detect_aruco_markers_1638781725_origin_051.png  }"
        "{img-ses  |       | Session id of detect aruco markers session, 1638781725 for example }"
        "{img-idx  |       | Starting index of detect aruco markers session, 51 for example }"
        "{dpi      |       | DPI for ids used for board }"
        "{w        |       | Number of squares in X direction }"
        "{h        |       | Number of squares in Y direction }"
        "{sl       |       | Square side length (in meters) }"
        "{ml       |       | Marker side length (in meters) }"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{c        |       | Output file with calibrated camera parameters }"
        "{dp       |       | File of marker detector parameters }"
        "{rs       |       | Apply refind strategy }";
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
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if (!parser.has("c") || !parser.has("img-path") || !parser.has("img-idx") || !parser.has("img-ses")) {
        cerr << "Not enought parameters specified" << endl;
        parser.printMessage();
        return 0;
    }

    int squaresX = parser.get<int>("w");
    int squaresY = parser.get<int>("h");
    float squareLength = parser.get<float>("sl") / (parser.has("dpi") ? parser.get<float>("dpi") : 1.0);
    float markerLength = parser.get<float>("ml") / (parser.has("dpi") ? parser.get<float>("dpi") : 1.0);
    int dictionaryId = parser.get<int>("d");
    bool refindStrategy = parser.has("rs");

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

    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    float axisLength = 0.5f * ((float)min(squaresX, squaresY) * (squareLength));

    // create charuco board object
    Ptr<aruco::CharucoBoard> charucoboard =
        aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
    Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

    double totalTime = 0;
    int totalIterations = 0;

    int img_idx = parser.get<int>("img-idx");
    int img_ses = parser.get<int>("img-ses");

    while(1) {
        Mat image;
        char path[1024];

        snprintf(path, sizeof(path), "%s\\detect_aruco_markers_%d_origin_%03d.png",
            parser.get<string>("img-path").c_str(), img_ses, img_idx);

        cerr << "Loading " << path;
        image = imread(path);
        if (!image.rows || !image.cols) {
            cerr << " Failed" << endl;
            break;
        };

        cerr << " OK" << endl;

        double tick = (double)getTickCount();

        vector< int > markerIds, charucoIds;
        vector< vector< Point2f > > markerCorners, rejectedMarkers;
        vector< Point2f > charucoCorners;
        Vec3d rvec, tvec;

        // detect markers
        aruco::detectMarkers(image, dictionary, markerCorners, markerIds, detectorParams,
                             rejectedMarkers);

        // refind strategy to detect more markers
        if(refindStrategy)
            aruco::refineDetectedMarkers(image, board, markerCorners, markerIds, rejectedMarkers,
                                         camMatrix, distCoeffs);

        // interpolate charuco corners
        int interpolatedCorners = 0;
        if(markerIds.size() > 0)
            interpolatedCorners =
                aruco::interpolateCornersCharuco(markerCorners, markerIds, image, charucoboard,
                                                 charucoCorners, charucoIds, camMatrix, distCoeffs);

        // estimate charuco board pose
        bool validPose = false;
        if(camMatrix.total() != 0)
            validPose = aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charucoboard,
                                                        camMatrix, distCoeffs, rvec, tvec);

        if (validPose)
        {
            cv::Mat V_charuco;

            // convert rvec/tvec to matrix
            wt_rvec_tvec_to_Mat4(rvec, tvec, V_charuco);

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
                1.0, 0.0, 0.0, squaresX * squareLength / 2.0,
                0.0, 1.0, 0.0, -squaresY * squareLength / 2.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0);

            V_cv = V_cv * TRANS_CENTR;

            snprintf(path, sizeof(path), "%s\\detect_aruco_markers_%d_pose_%03d.yaml",
                parser.get<string>("img-path").c_str(), img_ses, img_idx);

            cv::FileStorage fs(path, cv::FileStorage::WRITE);
            fs << "V_cv" << V_cv;
            fs.release();

            cerr << "V_cv[" << img_idx << "] = " << V_cv << endl;
        }

        img_idx++;
    }

    return 0;
}
