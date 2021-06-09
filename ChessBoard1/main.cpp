#include <stdio.h>

#include <quat.h>
#include <math.h>
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
#pragma comment(lib, "opencv_world3414d.lib")
#else
#pragma comment(lib, "opencv_world3414.lib")
#endif

int main(int argc, char** argv)
{
    cv::Mat frame, gray;
    std::string frame_file = "D:\\_manual_date\\2021\\2021-06-08\\ChessParser.git\\chess.2021-06-04\\01_00253.png";

    frame = cv::imread(frame_file);

    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    cv::findChessboardCorners
    (
        gray, cv::Size(), corners,
        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

    return 0;
}

