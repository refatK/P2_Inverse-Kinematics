#include "A2_2dlayer.h"

#include "../elements/circle.h"
#include "../elements/joint2d.h"

#include <iostream>

using namespace std;



A2_2DLayer::A2_2DLayer(OGLTWidget* parent)
    :OGLLayer(parent), m_UIMode(1), m_selected(0),
      m_solution(m_joints, m_links, m_obstacles)
{

}

void A2_2DLayer::mouse_hover(MouseInfo m){
    (void)m;
}

bool A2_2DLayer::setUIMode(int m){
    m_UIMode = m;
    return true;
}

bool A2_2DLayer::reset_view(){
    m_view.setToIdentity();
    return true;
}


void A2_2DLayer::key_press(KeyboardInfo ki){
    (void)ki;
}


void A2_2DLayer::key_release(KeyboardInfo ki){
    (void)ki;
}



void A2_2DLayer::scroll(double delta){
    (void)delta;
}



void A2_2DLayer::mouse_grab(MouseInfo m){

    if(m_UIMode==1){

        // Joint place / move mode
        m_selected = nullptr;
        Joint2D* selected_joint = nullptr;
        for(unsigned int i=0;i<m_joints.size();++i){
            if(m_joints[i]->m_is_movable && m_joints[i]->is_inside(m.pos)){
                selected_joint = m_joints[i];
                break;
            }
        }

        if (m.button == 0){
            if (!selected_joint){
                Joint2D* new_joint = new Joint2D(m_parent);
                new_joint->set_position(m.pos);
                m_joints.push_back(new_joint);
                m_selected = new_joint;
            } else {
                m_selected = selected_joint;
            }
        } else  if (m.button == 2 && selected_joint){
            for(auto it2 = selected_joint->get_links().begin(); it2 != selected_joint->get_links().end(); ++it2){
                for(auto it = m_links.begin(); it != m_links.end(); ++it){
                    if (*it == *it2){
                        m_links.erase(it);
                        break;
                    }
                }
            }
            for(auto it = m_joints.begin(); it != m_joints.end(); ++it){
                if (*it == selected_joint){
                    m_joints.erase(it);
                    break;
                }
            }
            delete selected_joint;
        }
    } else if(m_UIMode==2){

        if (m.button == 0){

            // Start/connect joints
            m_selected = nullptr;

            Joint2D* start_joint = nullptr;

            for(unsigned int i=0;i<m_joints.size();++i){
                if(m_joints[i]->m_is_movable && m_joints[i]->is_inside(m.pos)){
                    start_joint = m_joints[i];
                    break;
                }
            }

            if (start_joint != nullptr){
                Link2D* l = new Link2D(m_parent);
                l->set_first_joint(start_joint);
                l->compute(m.pos,100);
                m_links.push_back(l);
                m_selected = l;
            }


        } else if (m.button == 2){

            Link2D* link = nullptr;
            for(unsigned int i=0;i<m_links.size();++i){
                if (m_links[i]->m_is_movable && m_links[i]->is_inside(m.pos)){
                    link = m_links[i];
                    break;
                }
            }

            if (link != nullptr){
                // Remove link from joints
                link->get_first_joint()->remove_link(link);
                link->get_second_joint()->remove_link(link);

                for(auto it = m_links.begin(); it != m_links.end(); ++it){
                    if (*it == link){
                        m_links.erase(it);
                        break;
                    }
                }
                delete link;
            }
        }

    } else if(m_UIMode==3){

        // Obstacle place / move mode
        m_selected = nullptr;
        Obstacle2D* obstacle = nullptr;
        for(unsigned int i=0;i<m_obstacles.size();++i){
            if(m_obstacles[i]->is_inside(m.pos)){
                obstacle = m_obstacles[i];
                break;
            }
        }

        if (m.button == 0){
            if (!obstacle){
                Circle2D* c = new Circle2D(m_parent);
                c->compute(m.pos, 20, 100);
                c->m_color = QVector3D(1,0,0);
                m_obstacles.push_back((Obstacle2D*)c);
                m_selected = m_obstacles[m_obstacles.size()-1];
            } else {
                m_selected = obstacle;
            }

        } else if (m.button == 2 && obstacle){
            for(auto it = m_obstacles.begin(); it != m_obstacles.end(); ++it){
                if (*it == obstacle){
                    m_obstacles.erase(it);
                    break;
                }
            }
        }

    } else  if(m_UIMode==4){
        // Joint place / move mode
        m_selected = nullptr;
        for(unsigned int i=0;i<m_joints.size();++i){
            if(m_joints[i]->m_is_movable && m_joints[i]->is_inside(m.pos)){
                m_selected = m_joints[i];
                break;
            }
        }
    } else {
        cout<<"Warning: unknown mode! "<<m_UIMode<<endl;
    }

}


void A2_2DLayer::mouse_drag(MouseInfo m){

    if(m_UIMode==0){

    } else if(m_UIMode==1){
        if(m_selected){
            m_selected->mouse_drag(m);
        }
    } else if(m_UIMode==2){
        if(m_selected){
            ((Link2D*)m_selected)->compute(m.pos,100);
        }
    } else if(m_UIMode==3){
        if(m_selected){
            m_selected->mouse_drag(m);
        }
    }
    else if(m_UIMode==4){
         if(m_selected){
             m_solution.update(dynamic_cast<Joint2D*>(m_selected), m.pos);
             draw();
         }
    }

}


void A2_2DLayer::mouse_release(MouseInfo m){

    if(m_UIMode==0){


    } else if(m_UIMode==1){

        if(m_selected){

        }
    } else if(m_UIMode==2){

        if(m_selected){
            Joint2D* end_joint = nullptr;
            for(unsigned int i=0;i<m_joints.size();++i){
                if(m_joints[i]->m_is_movable && GeometryUtils::is_inside(m.pos.x(), m.pos.y(),m_joints[i]->getBB())){
                    end_joint = m_joints[i];
                    break;
                }
            }

            if (end_joint == ((Link2D*)m_selected)->get_first_joint() || end_joint == nullptr || ((Link2D*)m_selected)->get_first_joint() == end_joint)
            {
                // Invalid link, delete
                Link2D* l = m_links[m_links.size()-1];
                l->get_first_joint()->remove_link(l);
                m_links.pop_back();
                delete l;
            } else {
                // Otherwise add second joint to link
                ((Link2D*)m_selected)->set_second_joint(end_joint);
                ((Link2D*)m_selected)->compute(m.pos,100);
            }

        }

    }

    m_selected = 0;
}

void A2_2DLayer::mouse_double_click(MouseInfo m){

    if(m_UIMode==1){
        // Joint locking
        m_selected = 0;
        for(unsigned int i=0;i<m_joints.size();++i){
            if(GeometryUtils::is_inside(m.pos.x(), m.pos.y(),m_joints[i]->getBB())){
                m_joints[i]->set_locked(!this->m_joints[i]->is_locked());
                break;
            }
        }

    }
}


void A2_2DLayer::resizeGL(int w, int h){

    m_projection.setToIdentity();
    m_projection.ortho(0, w, h, 0, -1, 1);

}

bool A2_2DLayer::initializeGL(){

    reset_view();
    return true;
}


bool A2_2DLayer::draw(){

    if(!m_bIsInitGL){
        initializeGL();
        m_bIsInitGL = true;
    }


    // render all elememnts
    for(unsigned int i=0;i<m_joints.size();++i)
        m_joints[i]->draw(m_view, m_projection);
    for(unsigned int i=0;i<m_links.size();++i)
        m_links[i]->draw(m_view, m_projection);
    for(unsigned int i=0;i<m_obstacles.size();++i)
        m_obstacles[i]->draw(m_view, m_projection);


    return true;
}
