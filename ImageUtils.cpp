#include "ImageUtils.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>

QPixmap imgutils::matToPixmap(const cv::Mat &img)
{
    cv::Mat mat = img.clone();
    if (mat.channels() == 1)
        cv::cvtColor(mat, mat, cv::COLOR_GRAY2RGB);
    else if (mat.channels() == 4)
        cv::cvtColor(mat, mat, cv::COLOR_RGBA2RGB);
    QPixmap res = QPixmap::fromImage(QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888));
    return res;
}

void convertCylindricToSpherical(const cv::Mat &cyl, cv::Mat &sphere, float R)
{
    int w = cyl.cols, h = cyl.rows;
    float dphi = M_PI / h;
    float dlambda = M_PI / w;
    float phi, lambda, pixphi, pixlambda, z, f1, f2;
    float phig, lambdag;
    for (int x = R - 1; x > -R; x--) {
        for (int y = R - 1; y > -R; y--) {
            if (R * R - x * x - y * y < 0)
                continue;
            z = sqrt(R * R - x * x - y * y);
            phi = acos(y / R);
            phig = phi * 180 / M_PI;
            lambda = atan2(x, z) + M_PI_2;
            lambdag = lambda * 180 / M_PI;
            pixphi = phi / dphi;
            pixlambda = lambda / dlambda;

            // bilinear interpolation
            float x1 = floor(pixlambda), x2 = ceil(pixlambda), y1 = floor(pixphi), y2 = ceil(pixphi);
            if ( x2 - x1 < 0.5) {
                f1 = cyl.at<unsigned char>(y1, x1);
                f2 = cyl.at<unsigned char>(y2, x1);
            }
            else {
                f1 = (x2 - pixlambda) / (x2 - x1) * cyl.at<unsigned char>(y1, x1) +
                           (pixlambda - x1) / (x2 - x1) * cyl.at<unsigned char>(y2, x1);
                f2 = (x2 - pixlambda) / (x2 - x1) * cyl.at<unsigned char>(y1, x2) +
                           (pixlambda - x1) / (x2 - x1) * cyl.at<unsigned char>(y2, x2);
            }
            if (y2 - y1 > 0.5)
                sphere.at<unsigned char>(R - y, x + R) = (y2 - pixphi) / (y2 - y1) * f1 + (pixphi - y1) / (y2 - y1) * f2;
            else if (x2 - x1 > 0.5)
                sphere.at<unsigned char>(R - y, x + R) = f1;
        }
    }
}

void imgutils::convertCylindricToSphericalPair(const cv::Mat &cyl, cv::Mat &sphere_p, cv::Mat &sphere_m, float startAngle, float R)
{
    int w = cyl.cols, h = cyl.rows;
    sphere_p = cv::Mat(2 * R, 2 * R, cyl.type(), cv::Scalar(0));
    sphere_m = cv::Mat(2 * R, 2 * R, cyl.type(), cv::Scalar(0));
    cv::Mat twice(h, w * 3, cyl.type());
    cyl.copyTo(twice(cv::Rect(0, 0, w, h)));
    cyl.copyTo(twice(cv::Rect(w, 0, w, h)));
    cyl.copyTo(twice(cv::Rect(2 * w, 0, w, h)));
    cv::imwrite("twice.png", twice);

    int left = startAngle * w / 360 + w / 2 + w;
    cv::Mat mm = twice(cv::Rect(left, 0, w / 2, h)).clone();//twice(cv::Rect(w / 2, 0,w, h)).clone();
    cv::imwrite("mm.png", mm);
    convertCylindricToSpherical(mm, sphere_p, R);
    mm = twice(cv::Rect(left + w / 2, 0, w / 2, h)).clone();
    convertCylindricToSpherical(mm, sphere_m, R);    
}

std::vector<cv::DMatch> imgutils::calculateMatches(const cv::Mat &im1, const cv::Mat &im2,
                                                   std::vector<cv::Point2f> &pts1, std::vector<cv::Point2f> &pts2,
                                                   int features, float nndr, bool draw, QString name1, QString name2)
{
    pts1.clear();
    pts2.clear();
    std::vector<cv::KeyPoint> kp1, kp2;
    int minHessian = 300;
    cv::SurfFeatureDetector detector( minHessian );
    cv::Mat desc1, desc2;
    detector.detect( im1, kp1 );
    detector.detect( im2, kp2 );
    if (features == _SURF) {
        cv::SurfDescriptorExtractor extractor;
        extractor.compute( im1, kp1, desc1 );
        extractor.compute( im2, kp2, desc2 );
    }
    else {        
    }

    cv::BFMatcher matcher(cv::NORM_L2);
    std::vector<cv::DMatch> matches;
    std::vector< std::vector<cv::DMatch>> knnmatches;
    matcher.knnMatch( desc1, desc2, knnmatches, 2);    

    float filterRad2 = im1.cols / 2.0 * 0.9;
    filterRad2 *= filterRad2;
    unsigned char *used_pts2 = new unsigned char[kp2.size()];
    memset(used_pts2, 0, kp2.size());
    for (int i = knnmatches.size() - 1; i >= 0; i--) {
        if (knnmatches[i][0].distance <= nndr * knnmatches[i][1].distance) {
            if (used_pts2[knnmatches[i][0].trainIdx] == 0)
                used_pts2[knnmatches[i][0].trainIdx] = 1;
            else
                continue;
            cv::Point2f p1 = kp1[knnmatches[i][0].queryIdx].pt - cv::Point2f(im1.cols / 2, im1.rows / 2);
            cv::Point2f p2 = kp2[knnmatches[i][0].trainIdx].pt - cv::Point2f(im2.cols / 2, im2.rows / 2);
            if ((p1.x * p1.x + p1.y * p1.y) > filterRad2)
                continue;
            if ((p2.x * p2.x + p2.y * p2.y) > filterRad2)
                continue;
            matches.push_back(knnmatches[i][0]);
            pts1.push_back(p1);
            pts2.push_back(p2);
        }
    }
    delete[] used_pts2;
    if (draw) {
        cv::Mat out = drawMatchesOnImages(im1, im2, "matches", pts1, pts2);
        QString matchesImagePath = "matches_beforeFFiltering_" + name1 + "_" + name2 + ".png";
        cv::imwrite(matchesImagePath.toLatin1().data(), out);
    }
    cv::Mat mask;
    cv::Mat F = cv::findFundamentalMat(pts1, pts2, mask);
    for (int i = pts1.size() - 1; i >= 0; i--) {
        if (mask.at<unsigned char>(i, 0) == 0) {
            pts1.erase(pts1.begin() + i);
            pts2.erase(pts2.begin() + i);
        }
    }
    std::cerr << "pts1.size() = " << pts1.size();
    //-- Draw matches
    if (draw) {        
        cv::Mat out = drawMatchesOnImages(im1, im2, "matches", pts1, pts2);
        cv::waitKey(1);
        QString matchesImagePath = "matches_afterFFiltering_" + name1 + "_" + name2 + ".png";
        cv::imwrite(matchesImagePath.toLatin1().data(), out);
        cv::Mat out1, out2;
        cv::drawKeypoints(im1, kp1, out1);
        cv::drawKeypoints(im2, kp2, out2);
        cv::imwrite(QString("points_" + name1 + ".png").toLatin1().data(), out1);
        cv::imwrite(QString("points_" + name2 + ".png").toLatin1().data(), out2);
    }
    scalePoints(pts1, 0.001);
    scalePoints(pts2, 0.001);
    return matches;
}


cv::Mat imgutils::drawMatchesOnImages(const cv::Mat &im1, const cv::Mat &im2, const char *name, std::vector<cv::Point2f> pts1, std::vector<cv::Point2f> pts2)
{
    std::vector<cv::KeyPoint> kp1, kp2;
    std::vector<cv::DMatch> matches;
    cv::Point2f center(im1.cols / 2, im1.rows / 2);
    for (int i = 0; i < pts1.size(); i++) {
        cv::KeyPoint k1, k2;
        k1.pt = pts1[i] + center;
        k2.pt = pts2[i] + center;
        kp1.push_back(k1);
        kp2.push_back(k2);
        cv::DMatch match;
        match.queryIdx = i;
        match.trainIdx = i;
        matches.push_back(match);
    }
    cv::Mat out;
    cv::drawMatches(im1, kp1, im2, kp2, matches, out, cv::Scalar::all(-1), cv::Scalar::all(-1),
                std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    //cv::imshow(name, out);
    return out;
}


cv::Mat imgutils::createCameraMatrix(double f, cv::Size imgSize)
{
    double k[] = {f, 0,  imgSize.width / 2,
                  0, f, imgSize.height / 2,
                  0, 0,                1.0};
    return cv::Mat(3, 3, CV_64FC1, k).clone();
}


void imgutils::scalePoints(std::vector<cv::Point2f> &pts, float scaleFactor)
{
    for(auto it = pts.begin(); it != pts.end(); it++) {
        it->x = it->x * scaleFactor;
        it->y = it->y * scaleFactor;
    }
}
