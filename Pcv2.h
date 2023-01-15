#ifndef PCV2_H
#define PCV2_H

#include <opencv2/opencv.hpp>

namespace pcv2 {


    cv::Matx33f getCondition2D(const std::vector<cv::Vec3f> &points);


    cv::Mat_<float> getDesignMatrix_homography2D(const std::vector<cv::Vec3f> &conditioned_base,
                                                 const std::vector<cv::Vec3f> &conditioned_attach);


    cv::Matx33f solve_dlt_homography2D(const cv::Mat_<float> &A);


    cv::Matx33f decondition_homography2D(const cv::Matx33f &T_base, const cv::Matx33f &T_attach, const cv::Matx33f &H);


    cv::Matx33f homography2D(const std::vector<cv::Vec3f> &base, const std::vector<cv::Vec3f> &attach);


    enum GeometryType {
        GEOM_TYPE_POINT,
        GEOM_TYPE_LINE,
    };


    std::vector<cv::Vec3f>
    applyH_2D(const std::vector<cv::Vec3f> &geomObjects, const cv::Matx33f &H, GeometryType type);


    cv::Vec3f eucl2hom_point_2D(const cv::Vec2f &p);


}

#endif // PCV2_H
