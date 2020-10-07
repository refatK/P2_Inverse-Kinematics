#ifndef A2SOLUTION_H
#define A2SOLUTION_H

#include <vector>

#include "OpenGL/elements/joint2D.h"
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

};

#endif // A2SOLUTION_H
