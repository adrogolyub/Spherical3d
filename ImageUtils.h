#ifndef IMAGEUTILS
#define IMAGEUTILS

#include <QPixmap>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

const int _SURF = 0;

namespace imgutils
{
    // converts cv::Mat to QPixmap for showing on QLabel
    QPixmap matToPixmap(const cv::Mat &img);
    // convert cylinder-projected 360x180-degree image to image pair projected on western and eastern hemispheres
    // sphere_p has left border at startAngle degrees for longitude (X direction)
    void convertCylindricToSphericalPair(const cv::Mat &cyl, cv::Mat &sphere_p, cv::Mat &sphere_m, float startAngle = -90, float R = 500);
    // find matches for two images with SURF, filter by nndr and, optionally, draw them
    // pts1, pts2 are with respect to image center
    std::vector<cv::DMatch> calculateMatches(const cv::Mat &im1, const cv::Mat &im2,
                                             std::vector<cv::Point2f> &pts1, std::vector<cv::Point2f> &pts2, int features = _SURF,
                                             float nndr = 0.7, bool draw = true, QString name1 = "left", QString name2 = "right");

    cv::Mat drawMatchesOnImages(const cv::Mat &im1, const cv::Mat &im2, const char *name,
                             std::vector<cv::Point2f> pts1, std::vector<cv::Point2f> pts2);

    cv::Mat createCameraMatrix(double f, cv::Size imgSize);

    void scalePoints(std::vector<cv::Point2f> &pts, float scaleFactor);
}

#endif // IMAGEUTILS

