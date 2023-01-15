#include "Helper.h"

#include "Pcv2.h"

#include <vector>
#include <string>


namespace pcv2 {

    struct WinInfo {
        cv::Mat img;
        std::string name;
        std::vector<cv::Vec3f> pointList;
    };


    void getPointsCB(int event, int x, int y, int flags, void *param) {

        auto *win = (WinInfo *) param;

        switch (event) {
            case cv::EVENT_LBUTTONDOWN: {
                cv::Point2f p(x, y);
                cv::circle(win->img, p, 2, cv::Scalar(0, 255, 0), 2);
                cv::circle(win->img, p, 15, cv::Scalar(0, 255, 0), 2);
                cv::imshow(win->name, win->img);
                win->pointList.push_back(eucl2hom_point_2D({x + 0.5f, y + 0.5f}));
            }
                break;
        }
    }


    int getPoints(const cv::Mat &baseImg, const cv::Mat &attachImg, std::vector<cv::Vec3f> &points_base,
                  std::vector<cv::Vec3f> &points_attach) {
        std::cout << std::endl;
        std::cout << "Please select at least four points by clicking at the corresponding image positions:"
                  << std::endl;
        std::cout
                << "Firstly click at the point that shall be transformed (within the image to be attached), followed by a click on the corresponding point within the base image"
                << std::endl;
        std::cout << "Continue until you have collected as many point pairs as you wish" << std::endl;
        std::cout << "Stop the point selection by pressing any key" << std::endl << std::endl;


        WinInfo windowInfoBase = {
                baseImg.clone(),
                "Base image",
                {}
        };

        WinInfo windowInfoAttach = {
                attachImg.clone(),
                "Image to attach",
                {}
        };


        cv::namedWindow(windowInfoBase.name, 0);
        cv::imshow(windowInfoBase.name, windowInfoBase.img);
        cv::setMouseCallback(windowInfoBase.name, getPointsCB, (void *) &windowInfoBase);


        cv::namedWindow(windowInfoAttach.name, 0);
        cv::imshow(windowInfoAttach.name, windowInfoAttach.img);
        cv::setMouseCallback(windowInfoAttach.name, getPointsCB, (void *) &windowInfoAttach);

        cv::waitKey(0);

        cv::destroyWindow(windowInfoBase.name);
        cv::destroyWindow(windowInfoAttach.name);

        int numOfPoints = windowInfoBase.pointList.size();
        points_base = std::move(windowInfoBase.pointList);
        points_attach = std::move(windowInfoAttach.pointList);

        return numOfPoints;
    }


    cv::Mat stitch(const cv::Mat &base, const cv::Mat &attach, const cv::Matx33f &H) {
        cv::Mat corners(1, 4, CV_32FC2);
        corners.at<cv::Vec2f>(0, 0) = cv::Vec2f(0, 0);
        corners.at<cv::Vec2f>(0, 1) = cv::Vec2f(0, attach.rows);
        corners.at<cv::Vec2f>(0, 2) = cv::Vec2f(attach.cols, 0);
        corners.at<cv::Vec2f>(0, 3) = cv::Vec2f(attach.cols, attach.rows);
        perspectiveTransform(corners, corners, H);

        float x_start = std::min(std::min(corners.at<cv::Vec2f>(0, 0)[0], corners.at<cv::Vec2f>(0, 1)[0]), (float) 0);
        float x_end = std::max(std::max(corners.at<cv::Vec2f>(0, 2)[0], corners.at<cv::Vec2f>(0, 3)[0]),
                               (float) base.cols);
        float y_start = std::min(std::min(corners.at<cv::Vec2f>(0, 0)[1], corners.at<cv::Vec2f>(0, 2)[1]), (float) 0);
        float y_end = std::max(std::max(corners.at<cv::Vec2f>(0, 1)[1], corners.at<cv::Vec2f>(0, 3)[1]),
                               (float) base.rows);

        cv::Matx33f T = cv::Matx33f::zeros();
        T(0, 0) = 1;
        T(1, 1) = 1;
        T(2, 2) = 1;
        T(0, 2) = -x_start;
        T(1, 2) = -y_start;

        T = T * H;
        cv::Mat panorama;
        cv::warpPerspective(attach, panorama, T, cv::Size(x_end - x_start + 1, y_end - y_start + 1), cv::INTER_LINEAR);

        cv::Mat roi(panorama, cv::Rect(-x_start, -y_start, base.cols, base.rows));
        base.copyTo(roi, base);

        return panorama;
    }


}
