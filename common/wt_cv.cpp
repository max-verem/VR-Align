#include "wt_cv.h"

void wt_Mat4_to_row_matrix(const cv::Mat &M, q_matrix_type m)
{
    for (int j = 0; j < 4; j++) for (int i = 0; i < 4; i++) m[i][j] = M.at<double>(j, i);
};

void wt_row_matrix_to_Mat4(const q_matrix_type m, cv::Mat &M)
{
    M = cv::Mat::eye(4, 4, CV_64F);

    for (int j = 0; j < 4; j++) for (int i = 0; i < 4; i++) M.at<double>(j, i) = m[i][j];
};

void wt_quat_xyz_to_Mat4(const q_xyz_quat_type *pose, cv::Mat &V)
{
    q_matrix_type m;

    q_xyz_quat_to_row_matrix(m, pose);

    // copy CV matrix to q_matrix
    wt_row_matrix_to_Mat4(m, V);
};

void wt_Mat4_to_rvec_tvec(const cv::Mat &V, cv::Mat &rvec, cv::Mat &tvec)
{
    rvec = V(cv::Range(0, 3), cv::Range(0, 3));
    tvec = V(cv::Range(0, 3), cv::Range(3, 4));
};

void wt_rvec_tvec_to_Mat4(const cv::Mat &rvec, const cv::Mat &tvec, cv::Mat &V)
{
    cv::Mat R, t;

    // get rotation matrix
    if (rvec.size() == cv::Size(3, 3))
        R = rvec;
    else
        cv::Rodrigues(rvec, R);

    // create view matrix
    V = cv::Mat::eye(4, 4, R.type()); // T is 4x4
    V(cv::Range(0, 3), cv::Range(0, 3)) = R * 1; // copies R into T
    V(cv::Range(0, 3), cv::Range(3, 4)) = tvec * 1; // copies tvec into T
};

void wt_Mat4_to_quat_xyz(const cv::Mat &V, q_xyz_quat_type *pose)
{
    q_matrix_type m;

    // copy CV matrix to q_matrix
    wt_Mat4_to_row_matrix(V, m);

    // create pose (quat+xyz) from view matrix
    q_row_matrix_to_xyz_quat(pose, m);
}

void wt_rvec_tvec_to_quat_xyz(const cv::Mat &rvec, const cv::Mat &tvec, q_xyz_quat_type *pose)
{
    cv::Mat V;
    q_matrix_type m;

    wt_rvec_tvec_to_Mat4(rvec, tvec, V);

    // copy CV matrix to q_matrix
    wt_Mat4_to_row_matrix(V, m);

    // create pose (quat+xyz) from view matrix
    q_row_matrix_to_xyz_quat(pose, m);
};

void wt_rvec_tvec_to_Mat4(const cv::Vec3d &rvec, const cv::Vec3d &tvec, cv::Mat &V)
{
    cv::Mat _rvec = (cv::Mat_<double>(3, 1) << rvec[0], rvec[1], rvec[2]);
    cv::Mat _tvec = (cv::Mat_<double>(3, 1) << tvec[0], tvec[1], tvec[2]);
    wt_rvec_tvec_to_Mat4(_rvec, _tvec, V);
};

void wt_rvec_tvec_to_quat_xyz(const cv::Vec3d &rvec, const cv::Vec3d &tvec, q_xyz_quat_type *pose)
{
    cv::Mat _rvec = (cv::Mat_<double>(3, 1) << rvec[0], rvec[1], rvec[2]);
    cv::Mat _tvec = (cv::Mat_<double>(3, 1) << tvec[0], tvec[1], tvec[2]);
    wt_rvec_tvec_to_quat_xyz(_rvec, _tvec, pose);
}
