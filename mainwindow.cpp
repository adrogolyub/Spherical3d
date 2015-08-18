#include "mainwindow.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <QScrollBar>

#include <iostream>
#include <utility>

#include "ImageUtils.h"
#include "MathUtils.h"

using namespace cv;
using namespace imgutils;
using namespace mathutils;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent)
{
    setupUi(this);
    scene = new Scene3D();
    frameGL->layout()->addWidget(scene);
    scene->setPointSize(10);
    progressBar->hide();
    rbDAISY->hide();
    initModule_nonfree();    
}

void MainWindow::addlog(const QString &s)
{
    log->insertPlainText(s + "\n");
    qApp->processEvents();
}

void MainWindow::addlog(const QString &name, const Mat &m)
{
    QString s;
    for (int row = 0; row < m.rows; row++) {
        for (int col = 0; col < m.cols; col++) {
            s += QString::number(m.at<double>(row, col));
            if (col < m.cols - 1)
                s += " ";
        }
        if (row < m.rows - 1)
            s += "; ";
    }
    log->insertPlainText(name + ": [" + s + "]\n");
    qApp->processEvents();
}

void MainWindow::addlog(const QString &name, const Vec4d &v)
{
    QString s;
    for (int row = 0; row < v.rows; row++) {
        s += QString::number(v[row]);
        if (row < v.rows - 1)
            s += "; ";
    }
    log->insertPlainText(name + ": [" + s + "]\n");
    qApp->processEvents();
}

void dumpCameraPositionsToObj(const char *fileName, std::vector<cv::Mat> Ts)
{
    QFile F(fileName);
    F.open(QIODevice::WriteOnly);
    QTextStream T(&F);
    T << "# All coordinates are with respect to first camera\r\n";
    for (int i = 0; i < Ts.size(); i++) {
        double *m = (double*)Ts[i].data;
        T << QString("v %1 %2 %3\r\n")
             .arg(m[0])
             .arg(m[1])
             .arg(m[2])
             .replace(".", ",");
    }
    F.close();
}

// TODO: consider moving to QJSONDocument api
void dumpResultsToJSON(const char *fileName, QStringList inputFileNames, std::vector<cv::Mat> Rs, std::vector<cv::Mat> Ts)
{
    QFile F(fileName);
    F.open(QIODevice::WriteOnly);
    QTextStream T(&F);
    QStringList L;
    L << "{";
    for (int i = 0; i < Rs.size(); i++) {

        double *m = (double*)Rs[i].data;

        Vec4d Q = convertRotationMatrixToQuaternion(Rs[i]);

        L << QString("  \"%1\": {").arg(inputFileNames[i]);

        L << QString("    \"position\": {");
        L << QString("      \"x\": %1").arg(Ts[i].at<double>(0, 0));
        L << QString("      \"y\": %1").arg(Ts[i].at<double>(1, 0));
        L << QString("      \"z\": %1").arg(Ts[i].at<double>(2, 0));
        L << QString("    },");

        L << QString("    \"OZdirection\": {");
        L << QString("      \"x\": %1").arg(Q[1]);
        L << QString("      \"y\": %1").arg(Q[2]);
        L << QString("      \"z\": %1").arg(Q[3]);
        L << QString("      \"w\": %1").arg(Q[0]);
        L << QString("    },");

        L << QString("    \"OZrotation\": {");
        L << QString("      \"R_1\": %1 %2 %3").arg(m[0]).arg(m[1]).arg(m[2]);
        L << QString("      \"R_2\": %1 %2 %3").arg(m[3]).arg(m[4]).arg(m[5]);
        L << QString("      \"R_3\": %1 %2 %3").arg(m[6]).arg(m[7]).arg(m[8]);
        L << QString("    }");
        L << QString("  }");
    }
    L << "}";
    T << L.join("\r\n");
    F.close();    
}

void MainWindow::on_actionOpen_triggered()
{    
    imFiles = QFileDialog::getOpenFileNames(this, "Raw files");
    cmbImage->clear();
    cmbImage->addItems(imFiles);
    process();
    return;
}

void MainWindow::process()
{
    if (imFiles.size() < 2)
        return;
    scene->setCloud(PointCloud());
    progressBar->setValue(0);
    progressBar->show();
    qApp->processEvents();
    btnStart->setEnabled(false);
    const double scaling = 0.25;

    if (!log->toPlainText().isEmpty())
        addlog("\n====================================\n");

    bool isDebug = chkDebug->isChecked();
    int detector = _SURF;//rbSURF->isChecked() ? _SURF : _DAISY;

    int refimg = cmbImage->currentIndex();
    Mat image0 = imread(imFiles[refimg].toLatin1().data(), CV_LOAD_IMAGE_GRAYSCALE ); // reference image       

    scene->addPoint(CloudPoint(0, 0, 0, QColor(Qt::gray)));

    float startAngle0;
    float startAngleCur;

    std::vector<Mat> Rs, Ts;
    Rs.push_back(Mat::eye(3, 3, CV_64FC1));
    Ts.push_back(Mat::zeros(3, 1, CV_64FC1));
    int curimg = 0;

    addlog(QString("Processing %1 images.").arg(imFiles.size()));
    addlog("");
    addlog(QString("Reference image: ") + imFiles[refimg]);
    addlog("");    
    cv::Mat cumR = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat cumT = cv::Mat::zeros(3, 1, CV_64FC1);

    Qt::GlobalColor color = Qt::gray;

    for (int refimg = 0; refimg < imFiles.size() - 1; refimg++) {
        progressBar->setValue(100 / (imFiles.size() - 1) * curimg);
        curimg++;
        qApp->processEvents();
        int ind = refimg + 1;
        if (refimg == ind)
            continue;
        addlog(QString("Image %1: %2").arg(refimg).arg(imFiles[refimg]));
        addlog(QString("Image %1: %2").arg(curimg).arg(imFiles[ind]));        
        image0 = imread(imFiles[refimg].toLatin1().data(), CV_LOAD_IMAGE_GRAYSCALE ); // reference image
        Mat imageCur = imread(imFiles[ind].toLatin1().data(), CV_LOAD_IMAGE_GRAYSCALE );
        //cv::resize(imageCur, imageCur, Size(0, 0), scaling, scaling);
        std::vector<Point2f> pts0, ptsCur;

        startAngle0 = -90;
        startAngleCur = -90;

        QString name1 = imFiles[refimg].mid(imFiles[refimg].lastIndexOf("/") + 1, imFiles[refimg].lastIndexOf(".") - imFiles[refimg].lastIndexOf("/") - 1);
        QString name2 = imFiles[ind].mid(imFiles[ind].lastIndexOf("/") + 1, imFiles[ind].lastIndexOf(".") - imFiles[ind].lastIndexOf("/") - 1);

        // TODO: check that this is legal or just use source unstitched spherical (eqiangular) images
        // get optimal image hemisphere division for images according to matches
        if (chkCustomHemisphere->isChecked()) {                        
            Mat image0small, imageCursmall;
            cv::resize(image0, image0small, Size(), scaling, scaling, INTER_CUBIC);
            cv::resize(imageCur, imageCursmall, Size(), scaling, scaling, INTER_CUBIC);
            calculateMatches(image0small, imageCursmall, pts0, ptsCur, detector, sbNndr->value(), isDebug);
            cv::Mat mask;
            findFundamentalMat(pts0, ptsCur, mask);
            for (int i = pts0.size() - 1; i >= 0; i--) {
                if (mask.at<unsigned char>(i, 0) == 0) {
                    pts0.erase(pts0.begin() + i);
                    ptsCur.erase(ptsCur.begin() + i);
                }
            }
            // center of mass for matched points with respect to image center
            Point2f center0 = getCenterOfMass(pts0);
            Point2f centerCur = getCenterOfMass(ptsCur);

            // start angles for image positive hemispheres

            startAngle0 = (1000 * center0.x / image0small.cols) * 360 - 90;
            startAngleCur = (1000 * centerCur.x / imageCursmall.cols) * 360 - 90;
        }

        Mat sphere1p, sphere1m, sphere2m, sphere2p;
        convertCylindricToSphericalPair(image0, sphere1p, sphere1m, startAngle0);
        convertCylindricToSphericalPair(imageCur, sphere2p, sphere2m, startAngleCur);

        {
            imwrite(QString("sphere1m_%1.png").arg(name1).toLatin1().data(), sphere1m);
            imwrite(QString("sphere1p_%1.png").arg(name1).toLatin1().data(), sphere1p);
            imwrite(QString("sphere2m_%1.png").arg(name2).toLatin1().data(), sphere2m);
            imwrite(QString("sphere2p_%1.png").arg(name2).toLatin1().data(), sphere2p);
        }

        // get matches for spherical images
        if (rbForward->isChecked())
            calculateMatches(sphere1p, sphere2p, pts0, ptsCur, detector, sbNndr->value(), isDebug, name1, name2);
        else
            calculateMatches(sphere1m, sphere2m, pts0, ptsCur, detector, sbNndr->value(), isDebug, name1, name2);
        Mat D1, D2, D3;
        Mat iD1, iD2, iD3;
        if (pts0.size() == 0) {
            addlog("Failed to find correspondencies");
            continue;
        }
        addlog(QString("Found %1 good matches").arg(pts0.size()));
        // algebraic part to get list of camera model parameters candidates
        createQepMatrices(pts0, ptsCur, D1, D2, D3);
        createInvertedQepMatrices(D1, D2, D3, iD1, iD2, iD3);
        std::vector<Mat> eigvectors;

        auto AB = solveQepAndGetCandidateParameters(iD1, iD2, iD3, eigvectors); // list of prefiltered (a,b) candidate pairs
        cv::Mat E;
        double a, b;
        double err = estimateBestEssentialMatrix(pts0, ptsCur, AB, E, a, b);
        if (E.cols > 0 && cv::norm(E, NORM_L2) < 1e+5) {
            addlog("Estimated E", E);
            addlog(QString("E error is %1. Norm(E) = %2").arg(err).arg(cv::norm(E, NORM_L2)));
        }
        else {
            /*if (!rbForward->isChecked())
                calculateMatches(sphere1p, sphere2p, pts0, ptsCur, detector, sbNndr->value(), isDebug, name1, name2);
            else
                calculateMatches(sphere1m, sphere2m, pts0, ptsCur, detector, sbNndr->value(), isDebug, name1, name2);
            createQepMatrices(pts0, ptsCur, D1, D2, D3);
            createInvertedQepMatrices(D1, D2, D3, iD1, iD2, iD3);
            AB = solveQepAndGetCandidateParameters(iD1, iD2, iD3, eigvectors);
            err = estimateBestEssentialMatrix(pts0, ptsCur, AB, E, a, b);            
            if (E.cols > 0 || cv::norm(E, NORM_L2) < 1e+5) {
                color = Qt::blue;
            }
            else */{
                if (E.cols == 0)
                    addlog("Failed to estimate E");
                else
                    addlog(QString("E seems incorrect as norm(E) = %1").arg(cv::norm(E, NORM_L2)));
                color = (Qt::GlobalColor)(color + 3);
                scene->addPoint(CloudPoint(cumT.at<double>(0, 0),
                                           cumT.at<double>(1, 0),
                                           cumT.at<double>(2, 0),
                                           color));
                cumR = cv::Mat::eye(3, 3, CV_64FC1);
                addlog("");
                continue;
            }

        }

        int removed = removeOutliersUsingFundamentalMatrix(pts0, ptsCur, E, a, b);
        addlog(QString("Removed %1 outliers").arg(removed));
        err = calculateEpipolarErrorForEssentialMatrix(E, pts0, ptsCur, a, b);

      /*  if (chkCustomHemisphere->isChecked() && removed > 0) {
            // undistorted vectors
            std::vector<Point2f> outpts0, outptsCur;
            getUndistortedVectorsForEssentialMatrixEstimation(pts0, a, b, outpts0);
            getUndistortedVectorsForEssentialMatrixEstimation(ptsCur, a, b, outptsCur);
            cv::Mat E1;
            estimateEssentialMatrixFromPoints(outpts0, outptsCur, E1);
            addlog("Refined E", E1);
            double err1 = calculateEpipolarErrorForEssentialMatrix(E, pts0, ptsCur, a, b);
            if (err < err1)
                addlog(QString("Old E is better than new (%1 against %2). Switching back to old one").arg(err).arg(err1));
            else {
                addlog(QString("Refined E is better than old (%2 against %1). Switching to new one").arg(err).arg(err1));
                E = E1.clone();
            }
        }*/

        cv::Mat R, t;

        extractRotationAndTranslationFromEssentialMatrix(E, R, t);
        cumT = cumR.inv() * t + cumT;
        //cumT = cumT / cv::norm(cumT);
        cumR = R * cumR;
        addlog("R", R);
        addlog("t", t);
        addlog("cumT", cumT);
        cv::Vec4d q = convertRotationMatrixToQuaternion(R);
        addlog("Q", q);
        double ang, x, y, z;        
        convertQuaternionToAngleAndAxis(q, ang, x, y, z);
        addlog(QString("Rotation: %1, (%2, %3, %4)")
               .arg(ang * 180 / M_PI)
               .arg(x)
               .arg(y)
               .arg(z));

        q = convertRotationMatrixToQuaternion(cumR);
        convertQuaternionToAngleAndAxis(q, ang, x, y, z);
        addlog(QString("Rotation cum: %1, (%2, %3, %4)")
               .arg(ang * 180 / M_PI)
               .arg(x)
               .arg(y)
               .arg(z));

        Rs.push_back(cumR.clone());
        Ts.push_back(cumT.clone());

        scene->addPoint(CloudPoint(cumT.at<double>(0, 0),
                                   cumT.at<double>(1, 0),
                                   cumT.at<double>(2, 0),
                                   QColor(color)));
        addlog("");
    }

    QStringList L;
    foreach(QString s, imFiles) {
        QFileInfo inf(s);
        L << inf.fileName();
    }

    dumpResultsToJSON("scene_geometry.jsn", L, Rs, Ts);
    dumpCameraPositionsToObj("scene_geometry.obj", Ts);

    btnStart->setEnabled(true);
    progressBar->setValue(100);
    progressBar->hide();
    addlog("Calculation completed");
    if (log->verticalScrollBar())
        log->verticalScrollBar()->setValue(log->verticalScrollBar()->maximum());
}

void MainWindow::on_actionLoadObj_triggered()
{
    QString s = QFileDialog::getOpenFileName(this, QString("Load .obj file"), QString(), "Obj files (*.obj)");
    if (!s.isEmpty())
        loadObj(s);
}

void MainWindow::loadObj(QString path)
{
    QFile f(path);
    f.open(QIODevice::ReadOnly);
    QTextStream T(&f);
    T.readLine();
    PointCloud cloud;
    while (!T.atEnd()) {
        QString s = T.readLine();
        s.replace(",", ".");
        QStringList L = s.split(" ");
        cloud << CloudPoint(L[1].toFloat(), L[2].toFloat(), L[3].toFloat(), Qt::red);
    }
    f.close();
    scene->addCloud(cloud);
}

void MainWindow::on_btnStart_clicked()
{
    process();
}

void MainWindow::on_btnClear_clicked()
{
    log->clear();
    scene->setCloud(PointCloud());
}
