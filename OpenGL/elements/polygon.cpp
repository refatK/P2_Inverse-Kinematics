#include "polygon.h"

#include "../widgets/oglTwidget.h"

#include <iostream>

using namespace std;

Polygon2D::Polygon2D(OGLTWidget* parent)
    :DrawElement2D(parent), m_indexlineBuf(QOpenGLBuffer::IndexBuffer), m_thickness(10.0f)
{

}


void Polygon2D::mouse_grab(MouseInfo m){
    (void)m;
}


void Polygon2D::mouse_drag(MouseInfo m){
    for(unsigned int i=0;i<m_curve.size();++i){
        m_curve[i].position[0]+=m.pos[0]-m.pos0[0];
        m_curve[i].position[1]+=m.pos[1]-m.pos0[1];
    }
}


void Polygon2D::mouse_release(MouseInfo m){
    (void)m;
}


QRect Polygon2D::getBB()const{
    float minx=0, miny=0, maxx=0, maxy=0;
    GeometryUtils::findminmax(minx, miny,  maxx,  maxy, m_curve);

    QRect ret(minx, miny, maxx-minx, maxy-miny);
    return ret;
}

bool Polygon2D::is_inside(QVector2D pos)const{
    return GeometryUtils::is_inside(pos.x(), pos.y(),getBB());
}


bool Polygon2D::initializeGL(){


       if(!m_parent->context()){
           cout<<"Invalid call!!! - mess"<<endl;
           return false;
       }


       initializeOpenGLFunctions();

       if(!m_vao.create()){
           cout<<"Create buf error 10"<<endl;
           return false;
       }


       if(!m_lineBuf.create()){
           cout<<"Create buf error 11"<<endl;
           return false;
       }

       if(!m_indexlineBuf.create()){
           cout<<"Create buf error 12"<<endl;
           return false;
       }

    return true;
}


bool Polygon2D::draw(QMatrix4x4 model, QMatrix4x4 projection){

   // cout<<"poly"<<endl;

    if (this->m_bHidden)
    {
        return false;
    }

    if(!m_bIsInitGL){
        initializeGL();
        m_bIsInitGL = true;
    }

    // should onlhy call it when something is new
    //  if(m_bUpdateBuffer){
    updateOGLBuffer();
    //     m_bUpdateBuffer = false;
    // }

    ShaderManager& sm = m_parent->m_sm;
    QOpenGLShaderProgram* shader = sm.getProgram("line_shader");

    if(!shader){
        cout<<"Major error 12"<<endl;
        return false;
    }

    glDisable(GL_DEPTH_TEST);

    shader->bind();
    shader->setUniformValue("mvp_matrix", projection);
    shader->setUniformValue("texture", 0);
    shader->setUniformValue("color", m_color);

    quintptr offset = 0;
    int vertexLocation = shader->attributeLocation("a_position");
    shader->enableAttributeArray(vertexLocation);
    //shader->setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, sizeof(QVector3D));
    shader->setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, sizeof(VertexData));
    //shader->setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, offsetof(VertexData, position));

    // Offset for texture coordinate
    offset += sizeof(QVector3D);

    // Tell OpenGL programmable pipeline how to locate vertex texture coordinate data
    int texcoordLocation = shader->attributeLocation("a_texcoord");
    shader->enableAttributeArray(texcoordLocation);
    //shader->setAttributeBuffer(texcoordLocation, GL_FLOAT, offset, 2, sizeof(QVector2D));
    shader->setAttributeBuffer(texcoordLocation, GL_FLOAT, offset, 2, sizeof(VertexData));
    //shader->setAttributeBuffer(texcoordLocation, GL_FLOAT, offset, 2, offsetof(VertexData, texCoord));

    m_vao.bind();
    m_lineBuf.bind();
    m_indexlineBuf.bind();

    glLineWidth(m_thickness);
    //glLineWidth(10);

    int N = m_curve.size();
    if(N>0)
        glDrawElements(GL_LINE_STRIP, N, GL_UNSIGNED_SHORT, 0);


    return true;

}


bool Polygon2D::updateOGLBuffer(){

    // nothing to render
    if(m_curve.size()==0)
        return false;

    m_vao.bind();
    m_lineBuf.bind();
    m_lineBuf.allocate(&m_curve.front(), m_curve.size() * sizeof(VertexData));

    std::vector<GLushort> indices(m_curve.size());
    for(unsigned int j=0;j<indices.size();++j){
        indices[j] = j;
    }
    m_indexlineBuf.bind();
    m_indexlineBuf.allocate(&indices.front(), indices.size()*sizeof(GLushort));

    return true;
}


bool Polygon2D::save(QDataStream* out){

    return false;
}
bool Polygon2D::load(QDataStream* out){

    return false;
}
