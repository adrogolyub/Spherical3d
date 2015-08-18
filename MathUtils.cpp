#include "MathUtils.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include "dgegv.h"
#include "ImageUtils.h"

inline double f(double r, double a, double b)
{
    return r / tan(a * r / (1 + b * r * r));
}

inline double f(double u, double v, double a, double b)
{
    double r = sqrt(u * u + v * v);
    return f(r, a, b);
}

inline double f(double r, double a0)
{
    return r / (tan(a0 * r));
}

inline double f_a(double r, double a0)
{
    double t = tan(a0 * r);
    t = t * t;
    return -r * r * (1 + t) / t;
}

inline double f_b(double r, double a0)
{
    return -a0 * r * r * f_a(r, a0);
}

void mathutils::createQepMatrices(std::vector<cv::Point2f> pts1, std::vector<cv::Point2f> pts2,
                                  cv::Mat &D1, cv::Mat &D2, cv::Mat &D3)
{
    int N = pts1.size();
    D1 = cv::Mat(N, 15, CV_64FC1);
    D2 = cv::Mat(N, 15, CV_64FC1);
    D3 = cv::Mat(N, 15, CV_64FC1);
    // assume a 180-degree lens with radius of 50 mm
    const double a0 = 3.5;//M_PI / 0.05; // b0 = 0
    for (int i = 0; i < N; i++) {
        cv::Point2f p1 = pts1[i];
        cv::Point2f p2 = pts2[i];
        double u = (p1.x), u_ = (p2.x); // /1000.0
        double v = (p1.y), v_ = (p2.y); // /1000.0
        double r = sqrt(u * u + v * v), r_ = sqrt(u_ * u_ + v_ * v_);
        double w = f(r, a0) - a0 * f_a(r, a0), w_ = f(r_, a0) - a0 * f_a(r_, a0);
        double s = f_a(r, a0), s_ = f_a(r_, a0);
        double t = f_b(r, a0), t_ = f_b(r_, a0);
        double data1[] = {u * u_, v * u_, w * u_, u * v_,          v * v_,
                         w * v_, u * w_, v * w_, w * w_,          t * u_,
                         t * v_, u * t_, v * t_, t * w_ + w * t_, t * t_};
        double data2[] = {0,      0,      s * u_, 0,               0,
                         s * v_, u * s_, v * s_, s * w_ + w * s_, 0,
                         0,      0,      0,      t * s_ + s * t_, 0};
        double data3[] = {0, 0, 0, 0,      0,
                         0, 0, 0, s * s_, 0,
                         0, 0, 0, 0,      0};
        memcpy(D1.data + 15 * i * sizeof(double), data1, 15 * sizeof(double));
        memcpy(D2.data + 15 * i * sizeof(double), data2, 15 * sizeof(double));
        memcpy(D3.data + 15 * i * sizeof(double), data3, 15 * sizeof(double));
    }
}

void mathutils::createInvertedQepMatrices(const cv::Mat &D1, const cv::Mat &D2, const cv::Mat &D3, cv::Mat &iD1, cv::Mat &iD2, cv::Mat &iD3)
{
    cv::Mat D1t;
    cv::transpose(D1, D1t);
    iD1 = D1t * D3;
    iD2 = D1t * D2;
    iD3 = D1t * D1;
}

QString mathutils::outputMatricesToMatlabFormat(const cv::Mat &iD1, const cv::Mat &iD2, const cv::Mat &iD3)
{
    int ncols = iD1.cols;
    int nrows = iD1.rows;
    QString S1 = "iD1 = [", S2 = "iD2 = [", S3 = "iD3 = [";
    for (int row = 0; row < nrows; row++) {
        for (int col = 0; col < ncols; col++) {
            S1 += QString("%1 ").arg(((double*)iD1.data)[row * ncols + col]);
            S2 += QString("%1 ").arg(((double*)iD2.data)[row * ncols + col]);
            S3 += QString("%1 ").arg(((double*)iD3.data)[row * ncols + col]);
        }
        if (row != nrows - 1) {
            S1 += ";\n";
            S2 += ";\n";
            S3 += ";\n";
        }
    }
    S1 += "];";
    S2 += "];";
    S3 += "];";
    QString res = S1 + "\n" + S2 + "\n" + S3;
    return res;
}

void mathutils::getUndistortedVectorsForEssentialMatrixEstimation(std::vector<cv::Point2f> pts, double a, double b,
                                                     std::vector<cv::Point2f> &outpts)
{
    outpts.clear();
    double u, v, w, r;
    for (int i = 0; i < pts.size(); i++) {
        u = pts[i].x;
        v = pts[i].y;
        w = f(u, v, a, b);
        outpts.push_back(cv::Point2f(u / w, v / w));
    }
}

double mathutils::calculateEpipolarErrorForEssentialMatrix(const cv::Mat &F, std::vector<cv::Point2f> pts1, std::vector<cv::Point2f> pts2, double a, double b, double threshold)
{
    double error = 0, e;
    double maxE = 0;
    int maxEindex = -1;
    cv::Mat Ft;
    cv::transpose(F, Ft);
    double ff[3][3] = {F.at<double>(0, 0), F.at<double>(0, 1), F.at<double>(0, 2),
                       F.at<double>(1, 0), F.at<double>(1, 1), F.at<double>(1, 2),
                       F.at<double>(2, 0), F.at<double>(2, 1), F.at<double>(2, 2)};
    // omitting with degenerate cases
    if (fabs(ff[2][2]) < 0.5)
        return 1e+10;
    int good_points = 0;
    for (int i = 0; i < pts1.size(); i++) {
        double u1 = pts1[i].x;
        double u2 = pts2[i].x;

        double v1 = pts1[i].y;
        double v2 = pts2[i].y;

        double w1 = f(u1, v1, a, b);
        double w2 = f(u2, v2, a, b);

        cv::Mat p1(cv::Vec3d(u1, v1, w1), true);
        cv::Mat p2(cv::Vec3d(u2, v2, w2), true);

        cv::Mat p1t, p2t;

        cv::transpose(p1, p1t);
        cv::transpose(p2, p2t);

        cv::Mat Amat = p1t * Ft * F * p1 + p2t * F * Ft * p2;
        double A = Amat.at<double>(0, 0);
        cv::Mat Bmat = p2t * F * p1;
        double B = Bmat.at<double>(0, 0);
        B = B * B;
        double e = A / 2 - sqrt(A * A / 4 - B);        
        if (e > threshold)
            continue;
        if (e > maxE) {
            maxE = e;
            maxEindex = i;
        }
        good_points++;
        error += e * e;
    }
    // at least half points must be inliers
    if (good_points * 1.0 / pts1.size() < 0.5)
        return 1e+10;
    return error / good_points;
}

int mathutils::removeOutliersUsingFundamentalMatrix(std::vector<cv::Point2f> &pts1, std::vector<cv::Point2f> &pts2, const cv::Mat &F, double a, double b, double threshold)
{
    int removed = 0;
    cv::Mat Ft;
    cv::transpose(F, Ft);
    for (int i = pts1.size() - 1; i >= 0; i--) {
        double u1 = pts1[i].x;
        double u2 = pts2[i].x;

        double v1 = pts1[i].y;
        double v2 = pts2[i].y;

        double w1 = f(u1, v1, a, b);
        double w2 = f(u2, v2, a, b);

        cv::Mat p1(cv::Vec3d(u1, v1, w1), true);
        cv::Mat p2(cv::Vec3d(u2, v2, w2), true);

        cv::Mat p1t, p2t;

        cv::transpose(p1, p1t);
        cv::transpose(p2, p2t);

        cv::Mat Amat = p1t * Ft * F * p1 + p2t * F * Ft * p2;
        double A = Amat.at<double>(0, 0);
        cv::Mat Bmat = p2t * F * p1;
        double B = Bmat.at<double>(0, 0);
        B = B * B;
        double e = A / 2 - sqrt(A * A / 4 - B);
        if (e > threshold) {
            removed++;
            pts1.erase(pts1.begin() + i);
            pts2.erase(pts2.begin() + i);
        }
    }
    return removed;
}

int mathutils::removeOutliersUsingFundamentalMatrix(std::vector<cv::Point2f> &pts1, std::vector<cv::Point2f> &pts2, const cv::Mat &F, double threshold)
{
    int removed = 0;
    for (int i = pts1.size() - 1; i >= 0; i--) {
        double u1 = pts1[i].x;
        double u2 = pts2[i].x;

        double v1 = pts1[i].y;
        double v2 = pts2[i].y;

        double w1 = 1;
        double w2 = 1;

        cv::Mat p1(cv::Vec3d(u1, v1, w1), true);
        cv::Mat p2(cv::Vec3d(u2, v2, w2), true);

        cv::Mat p2t;

        cv::transpose(p2, p2t);

        cv::Mat Emat = p2t * F * p1;
        double e = Emat.at<double>(0, 0);
        if (e > threshold) {
            removed++;
            pts1.erase(pts1.begin() + i);
            pts2.erase(pts2.begin() + i);
        }
    }
    return removed;
}

void mathutils::estimateEssentialMatrixFromPoints(std::vector<cv::Point2f> pts1, std::vector<cv::Point2f> pts2, cv::Mat &E)
{
    //estimateEssentialMatrixFromPointsUsingFundamentalMatrix(pts1, pts2, E);
    E = cv::findFundamentalMat(pts1, pts2);
}

double mathutils::estimateBestEssentialMatrix(std::vector<cv::Point2f> pts1, std::vector<cv::Point2f> pts2, std::vector<std::pair<double, double> > ABsolutions, cv::Mat &bestE, double &bestA, double &bestB)
{
    std::vector<cv::Point2f> outpts1, outpts2;
    double minerror = 1e+10;
    for (int i = 0; i < ABsolutions.size(); i++) {
        double a = ABsolutions[i].first;
        double b = ABsolutions[i].second;
        getUndistortedVectorsForEssentialMatrixEstimation(pts1, a, b, outpts1);
        getUndistortedVectorsForEssentialMatrixEstimation(pts2, a, b, outpts2);

        double error;
        cv::Mat E;
        estimateEssentialMatrixFromPoints(outpts1, outpts2, E);
        error = calculateEpipolarErrorForEssentialMatrix(E, pts1, pts2, a, b);
        if (error < minerror) {
            bestE = E.clone();
            minerror = error;
            bestA = a;
            bestB = b;
        }
    }    
    return minerror;
}


void mathutils::extractRotationAndTranslationFromEssentialMatrix(const cv::Mat &E, cv::Mat &R, cv::Mat &T)
{
    cv::SVD svd;
    cv::Mat U, S, Vt, V, tx;
    svd.compute(E, S, U, Vt, cv::SVD::FULL_UV);
    cv::FileStorage file("svd.txt", cv::FileStorage::WRITE);    
    file << "u" << U;
    file << "vt" << Vt;
    file << "s" << S;

    cv::transpose(Vt, V);
    double dw[9] = {0, -1, 0, 1, 0, 0, 0, 0, 1};
    double dz[9] = {0, 1, 0, -1, 0, 0, 0, 0, 0};
    double dS[9] = {S.at<double>(0, 0), 0, 0, 0, S.at<double>(1, 0), 0, 0, 0, 0};
    cv::Mat W = cv::Mat(3, 3, CV_64FC1, dw).clone();
    cv::Mat W_;
    transpose(W, W_);
    R = U * W_ * Vt;
    cv::Mat Z(3, 3, CV_64FC1, dz);
    S = cv::Mat(3, 3, CV_64FC1, dS);
    // Theoretical formula, not fully applicable as E has non-equal singular values due to outliers
    // tx = V * W * S * Vt;
    // so we ommit W as if V has correct right singular vectors.
    // Actually, difference is nearly negligible for test images.
    // tx is a matrix representing vector multiplication with t
    tx = V * Z * Vt;
    cv::Vec3d t = cv::Vec3d(tx.at<double>(2, 1), tx.at<double>(0, 2), tx.at<double>(1, 0));
    cv::Vec3d tabs = cv::Vec3d(fabs(tx.at<double>(2, 1)), fabs(tx.at<double>(0, 2)), fabs(tx.at<double>(1, 0)));
    double mmin, mmax;
    int minIdx, maxIdx;
    cv::minMaxIdx(tabs, &mmin, &mmax, &minIdx, &maxIdx);
    T = cv::Mat(t) / norm(t);// t[maxIdx];
}

cv::Mat mathutils::getFundamentalMatrixFromEigenvector(const cv::Mat &h)
{
    return cv::Mat(3, 3, CV_64FC1, h.data).clone();
}

std::vector<std::pair<double, double> > mathutils::solveQepAndGetCandidateParameters(const cv::Mat &A0, const cv::Mat &A1, const cv::Mat &A2, std::vector<cv::Mat> &eigvectors)
{
    eigvectors.clear();
    // solve \lambda * Q1 * z = Q0 * z
    Eigen::MatrixXd Q0(30, 30);
    Eigen::MatrixXd Q1(30, 30);

    auto eA0 = Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>> ((double*)(A0.data));
    auto eA1 = Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>> ((double*)(A1.data));
    auto eA2 = Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>> ((double*)(A2.data));

    Q0 << eA0,
          Eigen::MatrixXd::Zero(15, 15),
          Eigen::MatrixXd::Zero(15, 15),
          Eigen::MatrixXd::Identity(15, 15);

    Q1 << -eA1,
          Eigen::MatrixXd::Identity(15, 15),
          -eA2,
          Eigen::MatrixXd::Zero(15, 15);

    std::vector<std::pair<double, double>> ABsolutions;

    double *A = new double[30 * 30];
    double *B = new double[30 * 30];

    for (int col = 0; col < 30; col++) {
        for (int row = 0; row < 30; row++) {
            A[row + col * 30] = Q0.coeff(row, col);
            B[row + col * 30] = Q1.coeff(row, col);
        }
    }

    double *eigenvalues = new double[30];
    double *eigenvectors = new double[30 * 30];
    int nreal = 0;
    dgegv(A, B, 30, eigenvalues, eigenvectors, nreal); // lapack-based generalized eigenvalue problem solution
    double norm = 0;
    double a, b;
    for (int i = 0; i < nreal; i++) {
        // minimum is 0.1 because only  0 < a < 10 are suggested valid
        if (fabs(eigenvalues[i]) > 0.1 && fabs(eigenvalues[i] < 1e+5)) {
            a = 1.0 / eigenvalues[i];
            b = eigenvectors[30 * i + 9] / eigenvectors[30 * i + 2];
            ABsolutions.push_back(std::make_pair(a, b));
            b = eigenvectors[30 * i + 10] / eigenvectors[30 * i + 5];
            ABsolutions.push_back(std::make_pair(a, b));
            b = eigenvectors[30 * i + 11] / eigenvectors[30 * i + 6];
            ABsolutions.push_back(std::make_pair(a, b));
            b = eigenvectors[30 * i + 12] / eigenvectors[30 * i + 7];
            ABsolutions.push_back(std::make_pair(a, b));
            b = eigenvectors[30 * i + 13] / eigenvectors[30 * i + 8];
            ABsolutions.push_back(std::make_pair(a, b));
            b = eigenvectors[30 * i + 14] / eigenvectors[30 * i + 13];
            ABsolutions.push_back(std::make_pair(a, b));

            cv::Mat h = cv::Mat(15, 1, CV_64FC1, eigenvectors + 30 * i).clone();
            eigvectors.push_back(h);
        }
    }
    return ABsolutions;
}

cv::Vec4d mathutils::convertRotationMatrixToQuaternion(const cv::Mat &R)
{
    double *m = (double*)R.data;
    double qx, qy, qz, qw;

    double tr = m[0] + m[4] + m[8];

    if (tr > 0) {
        double S = sqrt(tr + 1.0) * 2; // S=4*qw
        qw = 0.25 * S;
        qx = (m[7] - m[5]) / S;
        qy = (m[2] - m[6]) / S;
        qz = (m[3] - m[1]) / S;
    } else if ( m[0] > m[4] && m[0] > m[8] ) {
        double S = sqrt(1.0 + m[0] - m[4] - m[8]) * 2; // S=4*qx
        qw = (m[7] - m[5]) / S;
        qx = 0.25 * S;
        qy = (m[1] + m[3]) / S;
        qz = (m[2] + m[6]) / S;
    } else if (m[4] > m[8]) {
        double S = sqrt(1.0 + m[4] - m[0] - m[8]) * 2; // S=4*qy
        qw = (m[2] - m[6]) / S;
        qx = (m[1] + m[3]) / S;
        qy = 0.25 * S;
        qz = (m[5] + m[7]) / S;
    } else {
        double S = sqrt(1.0 + m[8] - m[0] - m[4]) * 2; // S=4*qz
        qw = (m[3] - m[1]) / S;
        qx = (m[2] + m[6]) / S;
        qy = (m[5] + m[7]) / S;
        qz = 0.25 * S;
    }

    return cv::Vec4d(qw, qx, qy, qz);
}

bool cmp_x(cv::Point2f a, cv::Point2f b)
{
    return a.x < b.x;
}

cv::Point2f mathutils::getCenterOfMass(const std::vector<cv::Point2f> &pts)
{
    // calculate median
    std::vector<cv::Point2f> tmp = pts;
    std::sort(tmp.begin(), tmp.end(), cmp_x);
    return tmp[tmp.size() / 2];

    float x = 0, y = 0;
    for (auto it = pts.begin(); it != pts.end(); it++) {
        x += it->x;
        y += it->y;
    }
    return cv::Point2f(x / pts.size(), y / pts.size());
}


void mathutils::convertQuaternionToAngleAndAxis(const cv::Vec4d q, double &angle, double &x, double &y, double &z)
{
    cv::Vec4d nq = q / cv::norm(q);
    double qx = nq[1], qy = nq[2], qz = nq[3], qw = nq[0];
    angle = 2 * acos(qw);

    double s = sqrt(1-qw*qw);
    if (s < 0.001) {
        // if s close to zero then direction of axis not important
        x = qx;
        y = qy;
        z = qz;
    }
    else {
        x = qx / s;
        y = qy / s;
        z = qz / s;
    }
}


void mathutils::estimateEssentialMatrixFromPointsUsingFundamentalMatrix(std::vector<cv::Point2f> pts1, std::vector<cv::Point2f> pts2, cv::Mat &E)
{
    double f = 8;
    cv::Size size(1000, 1000);
    cv::Mat K = imgutils::createCameraMatrix(f, size);
    cv::Mat mask;
    cv::Mat F = cv::findFundamentalMat(pts1, pts2, mask);
    E = K.t() * F * K;
}
