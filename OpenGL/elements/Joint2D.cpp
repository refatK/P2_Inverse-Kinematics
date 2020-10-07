#include "joint2D.h"
#include "link2d.h"

//#include "../oglTwidget.h"

#include <iostream>

#include <cmath>

using namespace std;


Joint2D::Joint2D(OGLTWidget* parent)
    :Polygon2D(parent), m_bIsLocked(false), m_pos(0,0), m_pos_locked(0,0)
{

    m_cross = new Cross2D(parent);
    m_cross->compute(QVector2D(0,0), 10, 100);
    m_cross->m_bHidden = true;

    m_circle = new Circle2D(parent);
    m_circle->compute(QVector2D(0,0), 10, 100);

    m_radius = 10;

}

Joint2D::~Joint2D(){

    // Delete visual elements
    delete m_cross;
    delete m_circle;

    // Delete links
    for(auto it = m_links.begin(); it != m_links.end(); ++it){
        // Remove from connecting joint's links
        Joint2D* other = ((*it)->get_other_joint(this));
        if (other != nullptr){
            ((*it)->get_other_joint(this))->remove_link(*it);
        }
        delete *it;
    }
}

void Joint2D::mouse_drag(MouseInfo m){

    //this->m_pos = m.pos;
    m_pos[0]+=m.pos[0]-m.pos0[0];
    m_pos[1]+=m.pos[1]-m.pos0[1];

    // Visual elements
    m_circle->mouse_drag(m);
    m_cross->mouse_drag(m);

    // Displace link
    for(auto it = m_links.begin(); it != m_links.end(); ++it){
        (*it)->compute(m_pos);
    }
}

void Joint2D::add_link(Link2D* link){
    m_links.push_back(link);
}

void Joint2D::remove_link(Link2D* link){
    for(auto it = m_links.begin(); it != m_links.end(); ++it){
        if (*it == link){
            m_links.erase(it);
            break;
        }
    }
}

std::vector<Joint2D*> Joint2D::get_children()
{
    std::vector<Joint2D*> children;
    for (Link2D* link: m_links){
        if (link->get_first_joint() == this){
            children.push_back(link->get_second_joint());
        }
    }
    return children;
}

std::vector<Joint2D*> Joint2D::get_parents()
{
    std::vector<Joint2D*> parents;
    for (Link2D* link: m_links){
        if (link->get_second_joint() == this){
            parents.push_back(link->get_first_joint());
        }
    }
    return parents;
}

void Joint2D::set_locked(bool value){

    if (!value && m_bIsLocked){
        // Unlock and remove X in middle
        m_bIsLocked = false;
        m_cross->m_bHidden = true;

    } else if (value && !m_bIsLocked) {
        // Lock and add X to middle
        m_bIsLocked = true;
        m_cross->m_bHidden = false;
        m_pos_locked = m_pos;
    }

}

void Joint2D::set_position(QVector2D pos){
    m_circle->compute(pos, 10, 100);
    m_cross->compute(pos, 10, 100);
    m_pos = pos;

    for(auto it = m_links.begin(); it != m_links.end(); ++it){
        (*it)->compute(pos);
    }
}

bool Joint2D::draw(QMatrix4x4 model, QMatrix4x4 projection){
    m_circle->draw(model, projection);
    m_cross->draw(model, projection);
}

QRect Joint2D::getBB()const{
    return m_circle->getBB();
}
