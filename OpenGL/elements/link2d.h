#ifndef LINK2D_H
#define LINK2D_H

#include "polygon.h"
#include "arrowhead2d.h"

class Joint2D;

class Link2D : public Polygon2D
{
public:
    Link2D(OGLTWidget* parent);

    virtual void mouse_grab(MouseInfo m);
    virtual void mouse_drag(MouseInfo m);
    virtual void mouse_release(MouseInfo m);
    virtual bool is_inside(QVector2D pos)const;
    virtual bool draw(QMatrix4x4 model, QMatrix4x4 projection);

    void compute(QVector2D pos, int n = 100);

    void set_first_joint(Joint2D* joint);
    void set_second_joint(Joint2D* joint);
    Joint2D* get_first_joint();
    Joint2D* get_second_joint();

    Joint2D* get_other_joint(Joint2D* joint);

    virtual ~Link2D();

private:
    Joint2D* m_joints[2];
    Arrowhead2D* m_arrowhead;

};

#endif // LINK2D_H
