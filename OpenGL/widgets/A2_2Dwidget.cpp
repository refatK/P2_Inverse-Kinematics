#include "A2_2Dwidget.h"

#include <QMouseEvent>

#include <math.h>

#include <iostream>


#include "../layers/A2_2dlayer.h"

using namespace std;



A2_2DWidget::A2_2DWidget(QWidget *parent) :
    OGLTWidget(parent)
{

   m_layers.push_back(new A2_2DLayer(this));

}

A2_2DWidget::~A2_2DWidget()
{

}

void A2_2DWidget::keyPressEvent(QKeyEvent *event){


    OGLTWidget::keyPressEvent(event);
}


void A2_2DWidget::keyReleaseEvent(QKeyEvent *event){


    OGLTWidget::keyReleaseEvent(event);
}


