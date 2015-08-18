#include <QMessageBox>
#include <QtGui>
#include <math.h>
#include <QDebug>
#include "scene3d.h"

const static float pi=3.141593, k=pi/180;

GLfloat VertexArray[1920*1280][3];
GLfloat ColorArray[1920*1280][3];
GLubyte IndexArray[1920*1280][3];

void Scene3D::initializeGL()
{   
   qglClearColor(Qt::white);
   glEnable(GL_DEPTH_TEST);
   glShadeModel(GL_FLAT);
   glEnable(GL_CULL_FACE);

   glEnableClientState(GL_VERTEX_ARRAY);
   glEnableClientState(GL_COLOR_ARRAY);
}

void Scene3D::resizeGL(int nWidth, int nHeight)
{
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();

   GLfloat ratio=(GLfloat)nHeight/(GLfloat)nWidth;

   /*if (nWidth>=nHeight)
      glOrtho(-1.0/ratio, 1.0/ratio, -1.0, 1.0, -10.0, 1.0);
   else
      glOrtho(-1.0, 1.0, -1.0*ratio, 1.0*ratio, -10.0, 1.0);*/

   glOrtho(-20.0, 20.0, -20, 20, -20.0, 20.0);

   glViewport(0, 0, (GLint)nWidth, (GLint)nHeight);
}

void Scene3D::paintGL()
{
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();         

  // glTranslatef(xTra, yTra, zTra);
   glScalef(nSca, nSca, nSca);
   
   /*glRotatef(xRot, 1, 0, 0);
   glRotatef(yRot, 0, 1, 0);
   glRotatef(zRot, 0, 0, 1);*/
   QMatrix4x4 m;
   m.rotate(trackball.rotation());
   glMultMatrixf(m.constData());

   drawAxis();
   drawFigure();
}

void Scene3D::setTranslation(float x, float y, float z)
{
	xTra = x;
	yTra = y;
	zTra = z;
	updateArrays();
	updateGL();
}

QPointF Scene3D::pixelPosToViewPos(const QPointF& p)
{
    return QPointF(2.0 * float(p.x()) / width() - 1.0,
                   1.0 - 2.0 * float(p.y()) / height());
}

void Scene3D::mousePressEvent(QMouseEvent* pe)
{
   if (pe->buttons() & Qt::LeftButton) {
       trackball.push(pixelPosToViewPos(pe->pos()), QQuaternion());
       pe->accept();
   }
}

void Scene3D::mouseReleaseEvent(QMouseEvent* pe)
{
    if (pe->button() == Qt::LeftButton) {
		trackball.release(pixelPosToViewPos(pe->pos()), QQuaternion());
        pe->accept();
    }
}

void Scene3D::mouseMoveEvent(QMouseEvent* pe)
{   
    if (pe->buttons() & Qt::LeftButton) {
		trackball.move(pixelPosToViewPos(pe->pos()), QQuaternion());
        pe->accept();
    } 
    updateGL();
}

void Scene3D::wheelEvent(QWheelEvent* pe)
{
   if ((pe->delta())>0) 
	   scale_plus(); 
   else if ((pe->delta())<0) 
	   scale_minus();

   updateGL();
}

void Scene3D::keyPressEvent(QKeyEvent* pe)
{
   switch (pe->key())
   {
      case Qt::Key_Plus:
         scale_plus();
      break;

      case Qt::Key_Equal:
         scale_plus();
      break;

      case Qt::Key_Minus:
         scale_minus();
      break;

      case Qt::Key_Up:
         rotate_up();
      break;

      case Qt::Key_Down:
         rotate_down();
      break;

      case Qt::Key_Left:
        rotate_left();
      break;

      case Qt::Key_Right:
         rotate_right();
      break;

      case Qt::Key_Z:
         translate_down();
      break;

      case Qt::Key_X:
         translate_up();
      break;

      case Qt::Key_Space:
         defaultScene();
      break;

      case Qt::Key_Escape:
         this->close();
      break;
   }

   updateGL();
}

void Scene3D::scale_plus()
{
   nSca = nSca*1.1;
}

void Scene3D::scale_minus()
{
   nSca = nSca/1.1;
}

void Scene3D::rotate_up()
{
   xRot += 1.0;
}

void Scene3D::rotate_down()
{
   xRot -= 1.0;
}

void Scene3D::rotate_left()
{
   zRot += 1.0;
}

void Scene3D::rotate_right()
{
   zRot -= 1.0;
}

void Scene3D::translate_down()
{
   zTra -= 0.05;
}

void Scene3D::translate_up()
{
   zTra += 0.05;
}

void Scene3D::defaultScene()
{
   xRot=-90; yRot=0; zRot=0; zTra=0; nSca=1; xTra = 0; yTra = 0;
}

void Scene3D::drawAxis()
{
   glLineWidth(3.0f);

   glColor4f(1.00f, 0.00f, 0.00f, 1.0f);
   glBegin(GL_LINES);
      glVertex3f( 1.0f,  0.0f,  0.0f);
      glVertex3f(-1.0f,  0.0f,  0.0f);
   glEnd();

   QColor halfGreen(0, 128, 0, 255);
   qglColor(halfGreen);
   glBegin(GL_LINES);
      glVertex3f( 0.0f,  1.0f,  0.0f);
      glVertex3f( 0.0f, -1.0f,  0.0f);

      glColor4f(0.00f, 0.00f, 1.00f, 1.0f);
      glVertex3f( 0.0f,  0.0f,  1.0f);
      glVertex3f( 0.0f,  0.0f, -1.0f);
   glEnd();
}

void Scene3D::drawFigure()
{
   glPointSize(pointSize);
   glVertexPointer(3, GL_FLOAT, 0, VertexArray);
   glColorPointer(3, GL_FLOAT, 0, ColorArray);
   glDrawArrays(GL_POINTS, 0, cloud.size());
}

void Scene3D::updateArrays()
{
    int N = cloud.size();
    for (int i = 0 ; i < N; ++i)
    {
        VertexArray[i][0] = cloud[i].x - xTra / 20.f;
		VertexArray[i][1] = cloud[i].y - yTra / 20.f;
		VertexArray[i][2] = cloud[i].z - zTra / 20.f;

        ColorArray[i][0] = cloud[i].c.red() / 255.f;
        ColorArray[i][1] = cloud[i].c.green() / 255.f;
        ColorArray[i][2] = cloud[i].c.blue() / 255.f;
    }
}

void Scene3D::createCloud()
{
    int N = 200;
    srand(rand());
	cloud.clear();
    for (int i = 0 ; i < N; ++i) {
		CloudPoint p;
        p.x = 2 * float(rand()) / RAND_MAX - 1;
        p.y = 2 * float(rand()) / RAND_MAX - 1;
        p.z = 2 * float(rand()) / RAND_MAX - 1;
		cloud << p;
    }
    updateGL();
}
