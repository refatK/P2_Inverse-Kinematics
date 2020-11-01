#ifndef OGLSPLINELAYER2D_H
#define OGLSPLINELAYER2D_H

#include "ogllayer.h"

#include "../elements/drawelement2D.h"

#include "../elements/joint2d.h"
#include "../elements/obstacle2d.h"
#include "../elements/link2d.h"

#include "../../solution/a2solution.h"

class A2_2DLayer : public OGLLayer
{
public:
    A2_2DLayer(OGLTWidget* parent);


    // UI
    virtual void mouse_grab(MouseInfo m);
    virtual void mouse_drag(MouseInfo m);
    virtual void mouse_release(MouseInfo m);
    virtual void mouse_double_click(MouseInfo m);

    virtual void key_press(KeyboardInfo ki);
    virtual void key_release(KeyboardInfo ki);

    virtual void scroll(double delta);

    virtual void mouse_hover(MouseInfo m);

    virtual bool setUIMode(int);
    int m_UIMode;// 0 = change the view, 1 = create and select Joints, 2 - create and remove links, 3 - create and select obstacles, 4 - showtime

    virtual bool reset_view();

    virtual void resizeGL(int w, int h);

    // virtual
    virtual bool initializeGL();
    virtual bool draw();

    virtual bool passMsg(TMessage*){return false;};

    virtual ~A2_2DLayer(){}

    // load / save later as it is trickier
    // not virtual because I can add a control mechanism to see if I can load/sav crrectly
    // control wor
    virtual bool save(QDataStream* out){(void)out; return false;};
    virtual bool load(QDataStream* out){(void)out; return false;};

    QMatrix4x4 m_projection, m_view;

    // Visible elements

    std::vector<Joint2D*> m_joints;
    std::vector<Link2D*> m_links;
    std::vector<Obstacle2D*> m_obstacles;

    DrawElement2D* m_selected = nullptr;

    // this is where the students add their solution
    A2Solution m_solution;

    };


#endif // OGLLAYER2D_H
