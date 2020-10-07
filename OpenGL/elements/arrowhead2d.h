#ifndef ARROWHEAD2D_H
#define ARROWHEAD2D_H

#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>

#include "polygon.h"

#include "../geometry/geometryutils.h"

class OGLTWidget;
class Arrowhead2D : public Polygon2D
{
public:
    Arrowhead2D(OGLTWidget* parent);


     // Virtual functions
    virtual void mouse_drag(MouseInfo m);

    void compute(QVector2D start, QVector2D end, float radius, int n=100);

    virtual ~Arrowhead2D(){}

    QVector2D m_center;
    float m_radius;

};

#endif // ARROWHEAD_H
