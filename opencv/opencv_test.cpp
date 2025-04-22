// opencv_test.cpp
#include "opencv_test.h"
#include <opencv2/opencv.hpp>
#include <iostream>

void opencv_test() {
    cv::Mat img(240, 320, CV_8UC3, cv::Scalar(0, 255, 0));  // 绿色图像
    cv::putText(img, "Hello OpenCV", cv::Point(50, 120),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);

    cv::imshow("Test", img);
    cv::waitKey(0);
}
