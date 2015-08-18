#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <QColor>
#include <QVector>

struct CloudPoint
{
    CloudPoint() { x = y = z = 0; }
    CloudPoint(float _x, float _y, float _z, QColor _c) { x = _x; y = _y; z = _z; c = _c; }
    float x;
    float y;
    float z;
    QColor c;
};

typedef QVector<CloudPoint> PointCloud;

#endif // POINTCLOUD_H
