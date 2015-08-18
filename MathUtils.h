#ifndef MATHUTILS_H
#define MATHUTILS_H

#include <opencv2/core/core.hpp>
#include <QString>

#define DEFAULT_THRESHOLD 0.001

namespace mathutils
{
    // creates matrices for initial Quadratic Eigenvalue Problem based on matches
    // using tangent-form for lens model f-function
    // u0, v0 - is a projection of principal point to image plane (e.g. image center)
    void createQepMatrices(std::vector<cv::Point2f> pts1, std::vector<cv::Point2f> pts2, cv::Mat &D1, cv::Mat &D2, cv::Mat &D3);

    // creates matrices for inverted QEP to deal with D3 singularity
    void createInvertedQepMatrices(const cv::Mat &D1, const cv::Mat &D2, const cv::Mat &D3, cv::Mat &iD1, cv::Mat &iD2, cv::Mat &iD3);

    QString outputMatricesToMatlabFormat(const cv::Mat &iD1, const cv::Mat &iD2, const cv::Mat &iD3);

    std::vector<std::pair<double, double>> solveQepAndGetCandidateParameters(const cv::Mat &A0, const cv::Mat &A1, const cv::Mat &A2, std::vector<cv::Mat> &eigvectors);

    // undistort point vectors to estimate essential matrix
    void getUndistortedVectorsForEssentialMatrixEstimation(std::vector<cv::Point2f> pts, double a, double b, std::vector<cv::Point2f> &outpts);

    // deviation from epipolar constraints using Oliensis approach ("Exact two-image structure from motion")
    double calculateEpipolarErrorForEssentialMatrix(const cv::Mat &F, std::vector<cv::Point2f> pts1, std::vector<cv::Point2f> pts2, double a, double b, double threshold = DEFAULT_THRESHOLD);

    void estimateEssentialMatrixFromPoints(std::vector<cv::Point2f> pts1, std::vector<cv::Point2f> pts2, cv::Mat &E);    

    void estimateEssentialMatrixFromPointsUsingFundamentalMatrix(std::vector<cv::Point2f> pts1, std::vector<cv::Point2f> pts2, cv::Mat &E);

    // estimates essential matrix with minimal reprojection error
    double estimateBestEssentialMatrix(std::vector<cv::Point2f> pts1, std::vector<cv::Point2f> pts2,
                                       std::vector<std::pair<double, double>> ABsolutions,
                                       cv::Mat &bestE, double &bestA, double &bestB);

    // uses SVD to estimate R and T
    // T is determined up to an unknown factor
    // and is constructed to have the largest value set to 1
    void extractRotationAndTranslationFromEssentialMatrix(const cv::Mat &E, cv::Mat &R, cv::Mat &T);

    // h is one of QEP solutions
    cv::Mat getFundamentalMatrixFromEigenvector(const cv::Mat &h);

    // remove outliers with undistorting vectors first using a, b
    int removeOutliersUsingFundamentalMatrix(std::vector<cv::Point2f> &pts1, std::vector<cv::Point2f> &pts2,
                                             const cv::Mat &F, double a, double b, double threshold = DEFAULT_THRESHOLD);

    // remove outliers without undistorting vectors
    int removeOutliersUsingFundamentalMatrix(std::vector<cv::Point2f> &pts1, std::vector<cv::Point2f> &pts2,
                                             const cv::Mat &F, double threshold = DEFAULT_THRESHOLD);

    cv::Vec4d convertRotationMatrixToQuaternion(const cv::Mat &R);

    void convertQuaternionToAngleAndAxis(const cv::Vec4d q, double &angle, double &x, double &y, double &z);

    cv::Point2f getCenterOfMass(const std::vector<cv::Point2f> &pts);
}

#endif // MATHUTILS_H

