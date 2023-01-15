#ifndef HELPER_H
#define HELPER_H


#include <opencv2/opencv.hpp>


namespace pcv2 {


    int getPoints(const cv::Mat &baseImg, const cv::Mat &attachImg, std::vector<cv::Vec3f> &points_base,
                  std::vector<cv::Vec3f> &points_attach);


    cv::Mat stitch(const cv::Mat &baseImg, const cv::Mat &attachImg, const cv::Matx33f &H);

}


#endif // HELPER_H
