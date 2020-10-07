#include "arrowhead2d.h"

#include "../widgets/oglTwidget.h"

#include <iostream>

#include <cmath>

using namespace std;

Arrowhead2D::Arrowhead2D(OGLTWidget* parent)
    :Polygon2D(parent), m_radius(0)
{
    m_thickness = 3;
}


void Arrowhead2D::mouse_drag(MouseInfo m){
    Polygon2D::mouse_drag(m);

    m_center[0]+=m.pos[0]-m.pos0[0];
    m_center[1]+=m.pos[1]-m.pos0[1];
}


void Arrowhead2D::compute(QVector2D start, QVector2D end, float radius, int n){

    m_center = end;
    m_radius = radius;
    m_curve.clear();

    QVector2D unit_dir = (end - start).normalized();
    QVector2D unit_flipped_top = QVector2D(unit_dir.y(), -unit_dir.x());
    QVector2D unit_flipped_bot = QVector2D(-unit_dir.y(), unit_dir.x());

    QVector2D arrow_top = (unit_flipped_top - unit_dir).normalized();
    QVector2D arrow_bot = (unit_flipped_bot - unit_dir).normalized();

    int div = n / 2;

    for(int i=0;i<n;++i){
        VertexData v;
        int rel = i % div;
        QVector2D q(0,0);

        if (i < 1*div){
            q = arrow_top;
            q = q * ((float)rel);
        } else if (i < 2*div){
            q = arrow_bot;
            q = q * ((float)rel);
        }
        //} else if (i < 3*div){
        //    q = arrow_bot;
        //    q = q * ((float)i/(float)rel);
        //}

        v.position = end + q * radius;

        m_curve.push_back(v);
    }

}
