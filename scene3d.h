#ifndef SCENE3D_H
#define SCENE3D_H

#include <QGLWidget>
#include <QList>
#include <QVector3D>

#include "PointCloud.h"
#include "trackball.h"

class Scene3D : public QGLWidget
{
    Q_OBJECT
public:
    Scene3D(QWidget* parent = 0) : QGLWidget(parent), trackball(0.05f, QVector3D(0, 1, 0), TrackBall::Sphere)
	{
		xRot=180; yRot=0; zRot=0; zTra=0; nSca=10; xTra = 0.0f; yTra = 0.0f;
	}

	void createCloud();

	void setCloud(PointCloud c) { cloud.clear(); addCloud(c); }
    void addCloud(PointCloud c) { cloud << c; updateArrays(); updateGL(); }
    void addPoint(CloudPoint p) { cloud << p; updateArrays(); updateGL(); }
	
	void setClipping(float minc, float maxc)
	{
		
	}

	void setPointSize(float s) { pointSize = s; updateGL(); }

	void setTranslation(float x, float y, float z);

private:
	float pointSize;
    TrackBall trackball;
	int w;
	int h;
    void updateArrays();
      GLfloat xRot;
      GLfloat yRot;
      GLfloat zRot;
      GLfloat xTra;
      GLfloat yTra;
      GLfloat zTra;
      GLfloat nSca;

      GLfloat **vertices;
      GLfloat **colors;
      GLubyte **indices;

      QPoint ptrMousePosition;

      void scale_plus();
      void scale_minus();
      void rotate_up();
      void rotate_down();
      void rotate_left();
      void rotate_right();
      void translate_down();
      void translate_up();
      void defaultScene();

      void drawAxis();

      void drawFigure();

      PointCloud cloud;

protected:
        QPointF pixelPosToViewPos(const QPointF& p);
      void initializeGL();
      void resizeGL(int nWidth, int nHeight);
      void paintGL();
      void mousePressEvent(QMouseEvent* pe);
      void mouseMoveEvent(QMouseEvent* pe);
      void mouseReleaseEvent(QMouseEvent* pe);
      void wheelEvent(QWheelEvent* pe);
      void keyPressEvent(QKeyEvent* pe);


};
#endif  // SCENE3D_H
