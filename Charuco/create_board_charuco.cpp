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
#include <opencv2/aruco/charuco.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

#ifdef _DEBUG
#pragma comment(lib, "opencv_world454d.lib")
#else
#pragma comment(lib, "opencv_world454.lib")
#endif

namespace {
const char* about = "Create a ChArUco board image";
const char* keys  =
        "{@outfile |<none> | Output image }"
        "{start    |       | Starting number for ids used }"
        "{w        |       | Number of squares in X direction }"
        "{h        |       | Number of squares in Y direction }"
        "{iw       |       | Output image width to override }"
        "{ih       |       | Output image height to override }"
        "{sl       |       | Square side length (in pixels) }"
        "{ml       |       | Marker side length (in pixels) }"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{m        |       | Margins size (in pixels). Default is (squareLength-markerLength) }"
        "{bb       | 1     | Number of bits in marker borders }"
        "{si       | false | show generated image }";
}

int main(int argc, char *argv[]) {
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(argc < 7) {
        parser.printMessage();
        return 0;
    }

    int iw = 0, ih = 0, start = 0;

    if (parser.has("start"))
        start = parser.get<int>("start");
    if (parser.has("iw"))
        iw = parser.get<int>("iw");
    if (parser.has("ih"))
        ih = parser.get<int>("ih");
    int squaresX = parser.get<int>("w");
    int squaresY = parser.get<int>("h");
    int squareLength = parser.get<int>("sl");
    int markerLength = parser.get<int>("ml");
    int dictionaryId = parser.get<int>("d");
    int margins = squareLength - markerLength;
    if(parser.has("m")) {
        margins = parser.get<int>("m");
    }

    int borderBits = parser.get<int>("bb");
    bool showImage = parser.get<bool>("si");

    cv::String out = parser.get<cv::String>(0);

    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    cv::Size imageSize;
    imageSize.width = squaresX * squareLength + 2 * margins;
    imageSize.height = squaresY * squareLength + 2 * margins;

    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(squaresX, squaresY, (float)squareLength,
        (float)markerLength, dictionary);

    if (start)
        for (int i = 0; i < board->ids.size(); i++)
            board->ids[i] += start;

    // show created board
    cv::Mat boardImage;
    board->draw(imageSize, boardImage, margins, borderBits);

    std::cout << "imageSize=" << imageSize << "\n";

    if (iw && ih)
    {
        cv::Mat dest;
        int h = (ih - imageSize.height) / 2, w = (iw - imageSize.width) / 2;

        std::cout << "iw=" << iw << ", ih=" << ih << ", w=" << w << ", h=" << h << "\n";

        cv::copyMakeBorder
        (
            boardImage, dest, h - 3, h - 3, w - 3, w - 3,
            cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255)
        );
        boardImage = dest;

        cv::cvtColor(boardImage, dest, cv::COLOR_GRAY2RGB);
        boardImage = dest;

        cv::copyMakeBorder(boardImage, dest, 1, 1, 1, 1, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 255));
        boardImage = dest;

        cv::copyMakeBorder(boardImage, dest, 1, 1, 1, 1, cv::BORDER_CONSTANT, cv::Scalar(0, 255, 0));
        boardImage = dest;

        cv::copyMakeBorder(boardImage, dest, 1, 1, 1, 1, cv::BORDER_CONSTANT, cv::Scalar(255, 0, 0));
        boardImage = dest;
    };


    if(showImage) {
        cv::imshow("board", boardImage);
        cv::waitKey(0);
    }

    cv::imwrite(out, boardImage);

    return 0;
}
