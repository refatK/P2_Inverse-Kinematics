#include "link2d.h"
#include "joint2d.h"
#include "arrowhead2d.h"

#include <cmath>
#include <QTransform>

Link2D::Link2D(OGLTWidget* parent)
    :Polygon2D(parent)
{
    m_joints[0] = nullptr;
    m_joints[1] = nullptr;
    m_thickness = 4;
    m_arrowhead = new Arrowhead2D(parent);
}


Link2D::~Link2D(){
    delete m_arrowhead;
}

void Link2D::mouse_drag(MouseInfo m){
    compute(m.pos, 100);

}

void Link2D::mouse_grab(MouseInfo m){
    (void) m;
}

void Link2D::mouse_release(MouseInfo m){
    (void) m;
}

bool Link2D::is_inside(QVector2D pos)const{


    if (!GeometryUtils::is_inside(pos.x(), pos.y(),getBB())){
        return false;
    }

    float a = m_joints[0]->m_pos.y()-m_joints[1]->m_pos.y();
    float b = m_joints[1]->m_pos.x()-m_joints[0]->m_pos.x();
    float c = (m_joints[0]->m_pos.x()-m_joints[1]->m_pos.x())*m_joints[0]->m_pos.y() + (m_joints[1]->m_pos.y()-m_joints[0]->m_pos.y())*m_joints[0]->m_pos.x();

    float distance = abs(a*pos.x()+ b*pos.y() + c)/sqrt(a*a + b*b);

    if (distance < (float)m_thickness){
        return true;
    } else {
        return false;
    }

}

void Link2D::set_first_joint(Joint2D* joint){

    m_joints[0] = joint;
    m_joints[0]->add_link(this);

}

void Link2D::set_second_joint(Joint2D* joint){

    m_joints[1] = joint;
    m_joints[1]->add_link(this);

}

Joint2D* Link2D::get_first_joint(){
    return m_joints[0];
}

Joint2D* Link2D::get_second_joint(){
    return m_joints[1];
}

Joint2D* Link2D::get_other_joint(Joint2D* joint){

    if (joint == m_joints[0]){
        return m_joints[1];
    } else if (joint == m_joints[1]){
        return m_joints[0];
    } else {
        return nullptr;
    }
}

void Link2D::compute(QVector2D pos, int n){

    if (m_joints[0] == nullptr)
        return;

    m_curve.clear();

    QVector2D pos1;
    QVector2D pos2;

    pos1 = m_joints[0]->get_position();
    if (m_joints[1] == nullptr)
    {
        pos2 = pos;
    } else {
        pos2 = m_joints[1]->get_position();
    }

    m_arrowhead->compute(pos1, pos2, 0.5f, 100);

    QVector2D diff = pos2 - pos1;

    for(int i=1;i<=n;++i){
        VertexData v;

        QVector2D newPos = pos1 + ((float)i / (float)n)*diff;

        v.position = newPos;

        m_curve.push_back(v);
    }
}

bool Link2D::draw(QMatrix4x4 model, QMatrix4x4 projection){
    Polygon2D::draw(model, projection);
    m_arrowhead->draw(model, projection);
    return true;
}


