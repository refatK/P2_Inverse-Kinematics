#include "a2solution.h"

#include "OpenGL/elements/link2d.h"
#include "OpenGL/elements/joint2d.h"
#include "OpenGL/elements/obstacle2d.h"

#include <QDebug>

#include <map>
#include <queue>
#include <algorithm>


using Eigen::MatrixXd;

A2Solution::A2Solution(std::vector<Joint2D*>& joints, std::vector<Link2D*>& links, std::vector<Obstacle2D*>& obstacles)
    :m_joints(joints), m_links(links), m_obstacles(obstacles){
}


void A2Solution::update(Joint2D* selected, QVector2D mouse_pos){
    std::cout << "\nNumber of Nodes: " << m_joints.size() << std::endl << std::endl;
    std::cout << ">NODE SELECTED: #" << getJointIndex(*selected) << std::endl;


    // Do Forward Kinematics
    this->doFkPass(*selected, mouse_pos);
}

void A2Solution::doFkPass(Joint2D& joint, QVector2D mouse_pos) {
    if (this->isRoot(joint)) {
        // When root is chosen, we should translate it and it's children
        QVector2D change = mouse_pos - joint.get_position();

        this->moveJointBy(joint, change);

    } else {
        // When non-root chosen, we rotate the selected the nodes and its children
        Joint2D* parent = joint.get_parents()[0];
        QVector2D parentPos = parent->get_position();
        QVector2D mathVecToJoint = this->qtToMathCoords(joint.get_position() - parentPos);
        QVector2D mathVecToMouse = this->qtToMathCoords(mouse_pos - parentPos);
        float theta = this->angleToRotate(mathVecToJoint, mathVecToMouse);

        this->rotateJointBy(joint, mathVecToJoint, theta);
    }
}

void A2Solution::moveJointBy(Joint2D& joint, QVector2D translation) {
    QVector2D curr_poss = joint.get_position();
    joint.set_position(curr_poss + translation);

    for (Joint2D* child : joint.get_children()) {
        moveJointBy(*child, translation);
    }
}

void A2Solution::rotateJointBy(Joint2D& joint, QVector2D currMathVecFromParent, float theta) {
    QVector2D currJointPos = joint.get_position();
    float mag = currMathVecFromParent.length();
    float currRot = this->getMathAngle(currMathVecFromParent);

    float newRot = currRot + theta;
    QVector2D newMathVecFromParent = QVector2D(mag*std::cos(newRot), mag*std::sin(newRot));
    QVector2D newQtVecFromParent = this->mathToQtCoords(newMathVecFromParent);

    joint.set_position(joint.get_parents()[0]->get_position() + newQtVecFromParent);

    for (Joint2D* child : joint.get_children()) {
        QVector2D mathVecFromJoint = this->qtToMathCoords(child->get_position() - currJointPos);
        rotateJointBy(*child, mathVecFromJoint, theta);
    }
}

float A2Solution::angleToRotate(QVector2D mathVecToJoint, QVector2D mathVecToNewPos) {
    float currAngleOfJoint = this->getMathAngle(mathVecToJoint);
    float newAngle = this->getMathAngle(mathVecToNewPos);
    return newAngle - currAngleOfJoint;
}

float A2Solution::getMathAngle(QVector2D mathVec) {
    return std::atan2(mathVec.y(), mathVec.x());
}

float A2Solution::radsToDegrees(float radians) {
    float pi = 4 * std::atan(1);
    return (radians * 180.0f) / pi;
}

QVector2D A2Solution::qtToMathCoords(QVector2D qtVec) {
    QVector2D mathVec = qtVec;
    mathVec.setY(-qtVec.y());
    return mathVec;
}

QVector2D A2Solution::mathToQtCoords(QVector2D mathVec) {
    QVector2D qtVec = mathVec;
    qtVec.setY(-mathVec.y());
    return qtVec;
}

bool A2Solution::isRoot(Joint2D& joint) {
    return joint.get_parents().empty();
}

int A2Solution::getJointIndex(Joint2D& joint) {
    for(int i=0; i < (int)this->m_joints.size(); ++i) {
        if (std::addressof(joint) == std::addressof(*m_joints[i])) {
            return i;
        }
    }

    std::cout << "SELECTED NODE IS NOT IN THE VECTOR.\n THIS SHOULDN'T HAPPEN." << std::endl;
    return -1;
}

// -----------------------------------------------------------------


void A2Solution::test_eigen_library(){

    // create a simple matrix 5 by 6
    MatrixXd mat(5,6);

    // Fills in matrix
    for(int row=0;row<mat.rows();row++){
        for(int col=0;col<mat.cols();col++){
            mat(row,col) = row+col;
        }
    }

    // create the pseudoinverse
    MatrixXd pseudo_inv = mat.completeOrthogonalDecomposition().pseudoInverse();

    // print the pseudoinverse
    std::cout << "--------------------------" << std::endl;
    std::cout << pseudo_inv << std::endl;
    std::cout << "--------------------------" << std::endl;

}
