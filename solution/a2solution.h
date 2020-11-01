#ifndef A2SOLUTION_H
#define A2SOLUTION_H

#include <vector>

#include "OpenGL/elements/joint2d.h"
#include "OpenGL/elements/obstacle2d.h"
#include "OpenGL/elements/link2d.h"

#include "dependencies/Eigen/Dense"
using Eigen::Vector2f;
using Eigen::Rotation2D;
using Eigen::MatrixXf;

class A2Solution
{
public:
    A2Solution(std::vector<Joint2D*>& joints, std::vector<Link2D*>& links, std::vector<Obstacle2D*>& obstacles);

    std::vector<Joint2D*>& m_joints;
    std::vector<Link2D*>& m_links;
    std::vector<Obstacle2D*>& m_obstacles;

    void update(Joint2D* selected, QVector2D mouse_pos);


    static void test_eigen_library();

private:
    void doFkPass(Joint2D& joint, QVector2D mouse_pos);
    void moveJointBy(Joint2D& joint, QVector2D translation);
    void rotateJointBy(Joint2D& joint, QVector2D currMathVecFromParent, float theta);
    float angleToRotate(QVector2D mathVecToJoint, QVector2D mathVecToNewPos);
    float getMathAngle(QVector2D mathVec);
    float radsToDegrees(float radians);
    QVector2D qtToMathCoords(QVector2D qtVec);
    QVector2D mathToQtCoords(QVector2D mathVec);
    bool isRoot(Joint2D& joint);
    int getJointIndex(Joint2D& joint);


};

#endif // A2SOLUTION_H
