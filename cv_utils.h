#include "K4aHelpers.h"
#include "k4arecord/record.h"
#include "k4abt.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <Body2D.h>

void ColorImageToMat(k4a_image_t c_image, cv::Mat& c_mat);
void  MatToColorImage(cv::Mat& c_mat, k4a_image_t& c_image, k4a_image_format_t format);
void Visualize2DBody(k4abt_frame_t& body_frame, cv::Mat& c_mat, k4a_calibration_t sensor_calibration);
void Visualize3DBody(k4abt_frame_t& body_frame, cv::Mat& d_mat, k4a_calibration_t sensor_calibration);