#ifndef BLOSSOM_POINT_H
#define BLOSSOM_POINT_H

#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>

#include "../elements/circle.h"
#include "../elements/cross.h"

#include "../geometry/geometryutils.h"

#include <vector>

class OGLTWidget;

class Link2D;

class Joint2D: public Polygon2D
{
public:
    Joint2D(OGLTWidget* parent);

    friend class Link2D;

     // Virtual functions
    virtual void mouse_drag(MouseInfo m);
    virtual bool draw(QMatrix4x4 model, QMatrix4x4 projection);
    virtual QRect getBB()const;

    void add_link(Link2D* link);
    void remove_link(Link2D* link);

    std::vector<Link2D*> get_links(){return m_links;}
    std::vector<Joint2D*> get_children();
    std::vector<Joint2D*> get_parents();

    bool is_locked(){return m_bIsLocked;}
    void set_locked(bool value);

    void set_position(QVector2D pos);
    QVector2D get_position(){return m_pos;}
    QVector2D get_position_locked(){return m_pos_locked;}

    float get_radius(){return m_radius;}

    virtual ~Joint2D();

private:

    // Visual elements
    Cross2D* m_cross;
    Circle2D* m_circle;

    // Connection elements
    QVector2D m_pos;
    QVector2D m_pos_locked;
    std::vector<Link2D*> m_links;

    bool m_bIsLocked;
    float m_radius;

};

#endif //
