#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "ui_mainwindow.h"
#include "scene3d.h"
#include <opencv2/core/core.hpp>

class MainWindow : public QMainWindow, private Ui::MainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
private:
    QStringList imFiles;
    Scene3D *scene;
private slots:
    void addlog(const QString &s);
    void addlog(const QString &name, const cv::Mat &m);
    void addlog(const QString &name, const cv::Vec4d &v);
    void on_actionOpen_triggered();
    void on_btnStart_clicked();
    void process();
    void on_actionLoadObj_triggered();
    void loadObj(QString path);
    void on_btnClear_clicked();
};

#endif // MAINWINDOW_H
