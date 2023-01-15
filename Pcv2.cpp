#include "Pcv2.h"

namespace pcv2 {


    cv::Matx33f getCondition2D(const std::vector<cv::Vec3f> &points) {
        float tx = 0.0f;
        float ty = 0.0f;
        float sx = 0.0f;
        float sy = 0.0f;
        cv::Matx33f T;

        for (const auto &point: points) {
            tx += point(0);
            ty += point(1);
        }
        tx /= points.size();
        ty /= points.size();

        for (const auto &point: points) {
            sx += abs(point(0) - tx);
            sy += abs(point(1) - ty);
        }
        sx /= points.size();
        sy /= points.size();

        T << 1 / sx, 0, -tx / sx, 0, 1 / sy, -ty / sy, 0, 0, 1;

        return T;
    }


    cv::Mat_<float> getDesignMatrix_homography2D(const std::vector<cv::Vec3f> &conditioned_base,
                                                 const std::vector<cv::Vec3f> &conditioned_attach) {
        cv::Mat_<float> A = cv::Mat_<float>::zeros(2 * conditioned_base.size(), 9);
        cv::Matx<float, 1, 3> A11;
        cv::Matx<float, 1, 3> A13;
        cv::Matx<float, 1, 3> A22;
        cv::Matx<float, 1, 3> A23;

        for (int i = 0; i < conditioned_base.size(); i++) {
            A11 = -conditioned_base[i](2) * conditioned_attach[i].t();
            A13 = conditioned_base[i](0) * conditioned_attach[i].t();
            A22 = -conditioned_base[i](2) * conditioned_attach[i].t();
            A23 = conditioned_base[i](1) * conditioned_attach[i].t();
            A(cv::Rect(0, i * 2, 3, 1)) << A11(0, 0), A11(0, 1), A11(0, 2);
            A(cv::Rect(6, i * 2, 3, 1)) << A13(0, 0), A13(0, 1), A13(0, 2);
            A(cv::Rect(3, i * 2 + 1, 3, 1)) << A22(0, 0), A22(0, 1), A22(0, 2);
            A(cv::Rect(6, i * 2 + 1, 3, 1)) << A23(0, 0), A23(0, 1), A23(0, 2);
        }

        return A;
    }


    cv::Matx33f solve_dlt_homography2D(const cv::Mat_<float> &A) {
        cv::Matx33f Hhat;
        cv::Mat_<float> U, W, VT;
        cv::SVD::compute(A, W, U, VT, 4);

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                Hhat(i, j) = VT(VT.rows - 1, i * 3 + j);
            }
        }

        return Hhat;
    }


    cv::Matx33f decondition_homography2D(const cv::Matx33f &T_base, const cv::Matx33f &T_attach, const cv::Matx33f &H) {
        cv::Matx33f Hout;
        Hout = T_base.inv() * H * T_attach;
        return Hout;
    }


    cv::Matx33f homography2D(const std::vector<cv::Vec3f> &base, const std::vector<cv::Vec3f> &attach) {
        std::vector<cv::Vec3f> conditioned_base;
        std::vector<cv::Vec3f> conditioned_attach;
        cv::Matx33f T_base;
        cv::Matx33f T_attach;
        cv::Mat_<float> A;
        cv::Matx33f Hhat;
        cv::Matx33f H;
        T_base = getCondition2D(base);
        T_attach = getCondition2D(attach);

        for (const auto &i: base) {
            conditioned_base.push_back(T_base * i);
        }
        for (const auto &i: attach) {
            conditioned_attach.push_back(T_attach * i);
        }

        A = getDesignMatrix_homography2D(conditioned_base, conditioned_attach);
        Hhat = solve_dlt_homography2D(A);
        H = decondition_homography2D(T_base, T_attach, Hhat);
        return H;
    }


    std::vector<cv::Vec3f>
    applyH_2D(const std::vector<cv::Vec3f> &geomObjects, const cv::Matx33f &H, GeometryType type) {
        std::vector<cv::Vec3f> result;

        switch (type) {
            case GEOM_TYPE_POINT: {
                for (const auto &geomObject: geomObjects) {
                    result.push_back(H * geomObject);
                }
            }
                break;
            case GEOM_TYPE_LINE: {
                for (const auto &geomObject: geomObjects) {
                    result.push_back((H.inv()).t() * geomObject);
                }
            }
                break;
            default:
                throw std::runtime_error("Unhandled geometry type!");
        }
        return result;
    }


    cv::Vec3f eucl2hom_point_2D(const cv::Vec2f &p) {
        cv::Vec3f P(p(0), p(1), 1);
        return P;
    }

}
