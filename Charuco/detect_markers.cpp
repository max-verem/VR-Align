#define _CRT_SECURE_NO_WARNINGS

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


#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <time.h>

#ifdef _DEBUG
#pragma comment(lib, "opencv_world454d.lib")
#else
#pragma comment(lib, "opencv_world454.lib")
#endif

#include <quat.h>
#include <vrpn_Connection.h>
#include <vrpn_Tracker.h>

#pragma comment(lib, "quatlib.lib")
#pragma comment(lib, "vrpn.lib")

static std::mutex vrpn_handle_tracker_mutex;

static void VRPN_CALLBACK vrpn_handle_tracker(void* userData, const vrpn_TRACKERCB t)
{
    std::lock_guard<std::mutex> guard(vrpn_handle_tracker_mutex);
    q_xyz_quat_type *pos = (q_xyz_quat_type*)userData;
    q_vec_copy(pos->xyz, t.pos);
    q_copy(pos->quat, t.quat);
};

static void vrpn_thread_proc(int* vrpn_thread_end, vrpn_Tracker_Remote* vrpn1, vrpn_Tracker_Remote* vrpn2, vrpn_Tracker_Remote* vrpn3)
{
    while (!(*vrpn_thread_end))
    {
        if (vrpn1) vrpn1->mainloop();
        if (vrpn2) vrpn2->mainloop();
        if (vrpn3) vrpn3->mainloop();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

using namespace std;
using namespace cv;

namespace {
const char* about = "Basic marker detection";
const char* keys  =
        "{vrpn-stk |       | Static tracker's VRPN address }"
        "{vrpn-mnt |       | Mounted tracker's VRPN address }"
        "{vrpn-cam |       | Recalculated camera postion VRPN address }"
        "{vm       |       | mode to select for cam, 1920x1080@25 for example}"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16,"
        "DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20}"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{c        |       | Camera intrinsic parameters. Needed for camera pose }"
        "{l        | 0.1   | Marker side length (in meters). Needed for correct scale in camera pose }"
        "{dp       |       | File of marker detector parameters }"
        "{r        |       | show rejected candidates too }"
        "{refine   |       | Corner refinement: CORNER_REFINE_NONE=0, CORNER_REFINE_SUBPIX=1,"
        "CORNER_REFINE_CONTOUR=2, CORNER_REFINE_APRILTAG=3}";
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



/**
 */
int main(int argc, char *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(argc < 2) {
        parser.printMessage();
        return 0;
    }

    int dictionaryId = parser.get<int>("d");
    bool showRejected = parser.has("r");
    bool estimatePose = parser.has("c");
    float markerLength = parser.get<float>("l");

    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    if(parser.has("dp")) {
        bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
        if(!readOk) {
            cerr << "Invalid detector parameters file" << endl;
            return 0;
        }
    }

    if (parser.has("refine")) {
        //override cornerRefinementMethod read from config file
        detectorParams->cornerRefinementMethod = parser.get<int>("refine");
    }
    std::cout << "Corner refinement method (0: None, 1: Subpixel, 2:contour, 3: AprilTag 2): " << detectorParams->cornerRefinementMethod << std::endl;

    int camId = parser.get<int>("ci");

    String video;
    if(parser.has("v")) {
        video = parser.get<String>("v");
    }

    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }

    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    Mat camMatrix, distCoeffs;
    if(estimatePose) {
        bool readOk = readCameraParameters(parser.get<string>("c"), camMatrix, distCoeffs);
        if(!readOk) {
            cerr << "Invalid camera file" << endl;
            return 0;
        }
    }

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

    vrpn_Tracker_Remote *vrpn_cam = NULL, *vrpn_mnt = NULL, *vrpn_stk = NULL;
    q_xyz_quat_type
        vrpn_pos_cam = { {0.0, 0.0, 0.0 }, {0.0, 0.0, 0.0, 1} },
        vrpn_pos_mnt = { {0.0, 0.0, 0.0 }, {0.0, 0.0, 0.0, 1} },
        vrpn_pos_stk = { {0.0, 0.0, 0.0 }, {0.0, 0.0, 0.0, 1} };

    if (parser.has("vrpn-stk"))
    {
        vrpn_stk = new vrpn_Tracker_Remote(parser.get<string>("vrpn-stk").c_str());
        vrpn_stk->register_change_handler(&vrpn_pos_stk, vrpn_handle_tracker);
    }

    if (parser.has("vrpn-mnt"))
    {
        vrpn_mnt = new vrpn_Tracker_Remote(parser.get<string>("vrpn-mnt").c_str());
        vrpn_mnt->register_change_handler(&vrpn_pos_mnt, vrpn_handle_tracker);
    }

    if (parser.has("vrpn-cam"))
    {
        vrpn_cam = new vrpn_Tracker_Remote(parser.get<string>("vrpn-cam").c_str());
        vrpn_cam->register_change_handler(&vrpn_pos_cam, vrpn_handle_tracker);
    }

    int vrpn_thread_end = 0;
    std::thread vrpn_thread_th(vrpn_thread_proc, &vrpn_thread_end, vrpn_stk, vrpn_mnt, vrpn_cam);

    double totalTime = 0;
    int totalIterations = 0;
    time_t start_time = time(NULL);
    int saved_images = 0;

    while(inputVideo.grab()) {
        Mat image, imageCopy;
        inputVideo.retrieve(image);

        double tick = (double)getTickCount();

        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;
        vector< Vec3d > rvecs, tvecs;

        // detect markers and estimate pose
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
        if(estimatePose && ids.size() > 0)
            aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs,
                                             tvecs);

        double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
        totalTime += currentTime;
        totalIterations++;
        if(totalIterations % 30 == 0) {
            cout << "Detection Time = " << currentTime * 1000 << " ms "
                 << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
        }

        // draw results
        image.copyTo(imageCopy);
        if(ids.size() > 0) {
            aruco::drawDetectedMarkers(imageCopy, corners, ids);

            if(estimatePose) {
                for(unsigned int i = 0; i < ids.size(); i++)
                    aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i],
                                    markerLength * 0.5f);
            }
        }

        if(showRejected && rejected.size() > 0)
            aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

        putText(imageCopy, "Press 's' to save current detection. 'ESC' to finish",
            Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);

        imshow("out", imageCopy);
        char key = (char)waitKey(waitTime);
        if(key == 27) break;
        if (key == 'S' || key == 's')
        {
            char tmp[1024];

            snprintf(tmp, sizeof(tmp), "detect_aruco_markers_%lld_origin_%03d.png", start_time, saved_images);
            imwrite(tmp, image);

            snprintf(tmp, sizeof(tmp), "detect_aruco_markers_%lld_detect_%03d.png", start_time, saved_images);
            imwrite(tmp, imageCopy);

            if (vrpn_stk || vrpn_cam || vrpn_mnt)
            {
                std::lock_guard<std::mutex> guard(vrpn_handle_tracker_mutex);

                cv::Mat v;
                snprintf(tmp, sizeof(tmp), "detect_aruco_markers_%lld_vrpn_%03d.yaml", start_time, saved_images);
                cv::FileStorage fs(tmp, cv::FileStorage::WRITE);

                cout << "Saving vrpn data " << tmp << " ...";

                v = (cv::Mat_<double>(3, 1) << vrpn_pos_cam.xyz[0], vrpn_pos_cam.xyz[1], vrpn_pos_cam.xyz[2]);
                fs << "vrpn_cam_xyz" << v;
                v = (cv::Mat_<double>(4, 1) << vrpn_pos_cam.quat[0], vrpn_pos_cam.quat[1], vrpn_pos_cam.quat[2], vrpn_pos_cam.quat[3]);
                fs << "vrpn_cam_quat" << v;

                v = (cv::Mat_<double>(3, 1) << vrpn_pos_stk.xyz[0], vrpn_pos_stk.xyz[1], vrpn_pos_stk.xyz[2]);
                fs << "vrpn_stk_xyz" << v;
                v = (cv::Mat_<double>(4, 1) << vrpn_pos_stk.quat[0], vrpn_pos_stk.quat[1], vrpn_pos_stk.quat[2], vrpn_pos_stk.quat[3]);
                fs << "vrpn_stk_quat" << v;

                v = (cv::Mat_<double>(3, 1) << vrpn_pos_mnt.xyz[0], vrpn_pos_mnt.xyz[1], vrpn_pos_mnt.xyz[2]);
                fs << "vrpn_mnt_xyz" << v;
                v = (cv::Mat_<double>(4, 1) << vrpn_pos_mnt.quat[0], vrpn_pos_mnt.quat[1], vrpn_pos_mnt.quat[2], vrpn_pos_mnt.quat[3]);
                fs << "vrpn_mnt_quat" << v;

                fs << "session" << (int)start_time;
                fs << "index" << saved_images;

                cout << " Done." << endl;
            }

            saved_images++;
        }
    }

    vrpn_thread_end = 1;
    vrpn_thread_th.join();

    return 0;
}
