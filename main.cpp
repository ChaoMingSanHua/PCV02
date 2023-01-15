#include "Pcv2.h"
#include "Helper.h"

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;


void run(const std::string &fnameBase, const std::string &fnameLeft, const std::string &fnameRight) {

    cv::Mat baseImage = cv::imread(fnameBase);
    cv::Mat attachImage = cv::imread(fnameLeft);
    if (!baseImage.data) {
        cerr << "ERROR: Cannot read image ( " << fnameBase << endl;
        cin.get();
        exit(-1);
    }
    if (!attachImage.data) {
        cerr << "ERROR: Cannot read image ( " << fnameLeft << endl;
        cin.get();
        exit(-1);
    }

    std::vector<cv::Vec3f> p_basis, p_attach;
    int numberOfPointPairs = pcv2::getPoints(baseImage, attachImage, p_basis, p_attach);

    cout << "Number of defined point pairs: " << numberOfPointPairs << endl;
    cout << endl << "Points in base image:" << endl;
    for (const auto &p: p_basis)
        cout << p << endl;
    cout << endl << "Points in second image:" << endl;
    for (const auto &p: p_attach)
        cout << p << endl;

    cv::Matx33f H = pcv2::homography2D(p_basis, p_attach);

    cv::Mat panorama = pcv2::stitch(baseImage, attachImage, H);

    const char *windowName = "Panorama";

    cv::namedWindow(windowName, 0);
    cv::imshow(windowName, panorama);
    cv::waitKey(0);
    cv::destroyWindow(windowName);

    baseImage = panorama;
    attachImage = cv::imread(fnameRight);
    if (!attachImage.data) {
        cout << "ERROR: Cannot read image ( " << fnameRight << " )" << endl;
        cin.get();
        exit(-1);
    }

    numberOfPointPairs = pcv2::getPoints(baseImage, attachImage, p_basis, p_attach);

    cout << "Number of defined point pairs: " << numberOfPointPairs << endl;
    cout << endl << "Points in base image:" << endl;
    for (const auto &p: p_basis)
        cout << p << endl;
    cout << endl << "Points in second image:" << endl;
    for (const auto &p: p_attach)
        cout << p << endl;


    H = pcv2::homography2D(p_basis, p_attach);

    panorama = pcv2::stitch(baseImage, attachImage, H);

    cv::namedWindow(windowName, 0);
    cv::imshow(windowName, panorama);
    cv::waitKey(0);
    cv::destroyWindow(windowName);

    cv::imwrite("panorama.png", panorama);
}


int main(int argc, char **argv) {

    string fnameBase, fnameLeft, fnameRight;

    if (argc != 4) {
        cout << "Usage: pcv2 <path to base image> <path to 2nd image> <path to 3rd image>" << endl;
        cout << "Press enter to continue..." << endl;
        cin.get();
        return -1;
    } else {
        fnameBase = argv[1];
        fnameLeft = argv[2];
        fnameRight = argv[3];
    }

    run(fnameBase, fnameLeft, fnameRight);

    cout << "Press enter to continue..." << endl;
    cin.get();

    return 0;

}
