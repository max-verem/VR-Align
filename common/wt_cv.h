#ifndef wt_cv_h
#define wt_cv_h

#include <quat.h>
#include <opencv2/opencv.hpp>

void wt_Mat4_to_row_matrix(const cv::Mat &M, q_matrix_type m);
void wt_row_matrix_to_Mat4(const q_matrix_type m, cv::Mat &M);

void wt_quat_xyz_to_Mat4(const q_xyz_quat_type *pose, cv::Mat &V);
void wt_Mat4_to_quat_xyz(const cv::Mat &V, q_xyz_quat_type *pose);

void wt_rvec_tvec_to_Mat4(const cv::Mat &rvec, const cv::Mat &tvec, cv::Mat &V);
void wt_rvec_tvec_to_Mat4(const cv::Vec3d &rvec, const cv::Vec3d &tvec, cv::Mat &V);

void wt_rvec_tvec_to_quat_xyz(const cv::Mat &rvec, const cv::Mat &tvec, q_xyz_quat_type *pose);
void wt_rvec_tvec_to_quat_xyz(const cv::Vec3d &rvec, const cv::Vec3d &tvec, q_xyz_quat_type *pose);

void wt_Mat4_to_rvec_tvec(const cv::Mat &V, cv::Mat &rvec, cv::Mat &tvec);

#endif