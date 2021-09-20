#ifndef wt_cv_h
#define wt_cv_h

#include <quat.h>
#include <opencv2/opencv.hpp>

void wt_Mat4_to_row_matrix(cv::Mat &M, q_matrix_type m);
void wt_row_matrix_to_Mat4(q_matrix_type m, cv::Mat &M);
void wt_rvec_tvec_to_quat_xyz(cv::Mat &rvec, cv::Mat &tvec, q_xyz_quat_type *pose);

#endif