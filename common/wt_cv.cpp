#include "wt_cv.h"

void wt_Mat4_to_row_matrix(cv::Mat &M, q_matrix_type m)
{
    for (int j = 0; j < 4; j++) for (int i = 0; i < 4; i++) m[i][j] = M.at<double>(j, i);
};

void wt_row_matrix_to_Mat4(q_matrix_type m, cv::Mat &M)
{
    for (int j = 0; j < 4; j++) for (int i = 0; i < 4; i++) M.at<double>(j, i) = m[i][j];
};

void wt_rvec_tvec_to_quat_xyz(cv::Mat &rvec, cv::Mat &tvec, q_xyz_quat_type *pose)
{
    cv::Mat R, t, V;
    q_matrix_type m;

    // get rotation matrix
    if (rvec.size() == cv::Size(3, 3))
        R = rvec;
    else
        cv::Rodrigues(rvec, R);

    // create view matrix
    V = cv::Mat::eye(4, 4, R.type()); // T is 4x4
    V(cv::Range(0, 3), cv::Range(0, 3)) = R * 1; // copies R into T
    V(cv::Range(0, 3), cv::Range(3, 4)) = tvec * 1; // copies tvec into T

    // copy CV matrix to q_matrix
    wt_Mat4_to_row_matrix(V, m);

    // create pose (quat+xyz) from view matrix
    q_row_matrix_to_xyz_quat(pose, m);
};

