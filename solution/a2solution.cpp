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

    // selected must be locked, else exit
    if (!selected->is_locked()) {
        return;
    }

    // clear data
    this->m_root = nullptr;
    this->m_selected_index = -1;
    this->m_used_joints.clear();
    this->pos_used_joints.clear();
    this->m_locked_joints.clear();
    this->pos_locked_joints.clear();

    // setup data
    this->setRoot(); // TODO: currently just takes first root it sees, not based on selected
    this->setRelevantJoints(selected);

    MatrixXf jacobian;
    VectorXf errorVector;
    VectorXf deltaTheta;

//    for (int i=0; i<1; ++i) {
    for (int i=0; i<this->maxIterations; ++i) {
        // get needed values
        jacobian = this->createJacobian(this->m_used_joints, this->pos_used_joints, this->m_locked_joints, this->pos_locked_joints, this->epsilon);
        errorVector = this->createErrorVec(this->m_locked_joints, this->pos_locked_joints, *selected, mouse_pos);
        errorVector = this->beta * errorVector;

        // do DLS
        deltaTheta = this->doDls(jacobian, errorVector, this->lambda);

        // do FK with new angles (TODO: can maybe save and return old poses if need them, like for collisions)
        this->doFkPassWithChanges(this->m_used_joints, this->pos_used_joints, deltaTheta);

        // check if close enough, in which case, done calculations
        float errorMag = (pos_used_joints[this->m_selected_index] - mouse_pos).length();
        if (errorMag < this->inRangeMag) {
            break;
        }
    }

    std::cout << "--------------------------" << std::endl;
    std::cout << deltaTheta << std::endl;
    std::cout << "--------------------------" << std::endl;

    std::cout << "--------------------------" << std::endl;
    std::cout << jacobian << std::endl;
    std::cout << "--------------------------" << std::endl;

    // Update Values
    this->updatePositionsInUi(this->m_used_joints, this->pos_used_joints);
}

void A2Solution::updatePositionsInUi(std::vector<Joint2D*>& allJointsToUpdate, std::vector<QVector2D>& newPosAllJoints) {
    for (int i=0; i<allJointsToUpdate.size(); ++i) {
        allJointsToUpdate[i]->set_position(newPosAllJoints[i]);
    }
}


std::vector<QVector2D> A2Solution::doFkPassWithChanges(std::vector<Joint2D*>& allJoints, std::vector<QVector2D>& posAllJoints, VectorXf& deltaTheta) {
    // changes at 0 and 1 are the translations of the root
    // TODO: ignore root for now

    // rotate the joints
    for (int i=0; i<allJoints.size(); ++i) {
        if (i == 0) { continue; } // ignore root
        int row = i+1;

        int parentIndex = this->getParentIndex(allJoints, i);
        float theta = deltaTheta[row];
        QVector2D mathVecFromParent = this->qtToMathCoords(posAllJoints[i] - posAllJoints[parentIndex]);

        this->rotateJointByNoUpdate(allJoints, posAllJoints, i, parentIndex, mathVecFromParent, theta);
    }

    return posAllJoints;
}

void A2Solution::rotateJointByNoUpdate(std::vector<Joint2D*>& allJoints, std::vector<QVector2D>& posAllJoints, int currIndex, int parentIndex, QVector2D currMathVecFromParent, float theta) {
    QVector2D posBeforeUpdate = posAllJoints[currIndex];
    float mag = currMathVecFromParent.length();
    float currRot = this->getMathAngle(currMathVecFromParent);

    float newRot = currRot + theta;
    QVector2D newMathVecFromParent = QVector2D(mag*std::cos(newRot), mag*std::sin(newRot));
    QVector2D newQtVecFromParent = this->mathToQtCoords(newMathVecFromParent);

    // Officially update the position
    posAllJoints[currIndex] = posAllJoints[parentIndex] + newQtVecFromParent;

//    for (Joint2D* child : allJoints[currIndex]->get_children()) {
//        QVector2D mathVecFromJoint = this->qtToMathCoords(child->get_position() - posBeforeUpdate);
//        rotateJointBy(*child, mathVecFromJoint, theta);
//    }

    for (int childIndex : this->getChildIndexes(allJoints, currIndex)) {
        QVector2D mathVecFromJoint = this->qtToMathCoords(posAllJoints[childIndex] - posBeforeUpdate);
        rotateJointByNoUpdate(allJoints, posAllJoints, childIndex, currIndex, mathVecFromJoint, theta);
    }
}

VectorXf A2Solution::doDls(MatrixXf j, VectorXf e, float lambda) {
    MatrixXf jT = j.transpose();
    MatrixXf i = MatrixXf::Identity(j.rows(), j.rows());
    float lamSq = lambda*lambda;

    MatrixXf dls = jT * (j*jT + lamSq*i).inverse();
    return dls*e;
}

VectorXf A2Solution::createErrorVec(std::vector<Joint2D*>& lockedJoints, std::vector<QVector2D*>& posLockedJoints, Joint2D& selected, QVector2D& expectedPos) {
    int numLocked = lockedJoints.size();
    int numRows = 2 * numLocked;

    VectorXf errorVec(numRows);

    for (int row=0; row<numRows; row=row+2) {
        int currIndex = row/2;
        Joint2D* current = lockedJoints[currIndex];
        Vector3f currMath = this->qtToEigenMath(*posLockedJoints[currIndex]);

        if (current == &selected) {
            Vector3f expectedMath = this->qtToEigenMath(expectedPos);
            Vector3f error = expectedMath - currMath;
            errorVec(row) = error.x();
            errorVec(row+1) = error.y();
        } else {
            // TODO: need to store initial postion and current position, just 0 for now
            // I think done now because of posVectors
            errorVec(row) = 0;
            errorVec(row+1) = 0;
        }
    }

    return errorVec;
}

MatrixXf A2Solution::createJacobian(std::vector<Joint2D*>& allJoints, std::vector<QVector2D>& posAllJoints, std::vector<Joint2D*>& lockedJoints, std::vector<QVector2D*>& posLockedJoints, float epsilon) {
    int numJoints = allJoints.size();
    int numLocked = lockedJoints.size();

    int numRows = 2 * numLocked;
    int numCols = numJoints + 1; // The plus one is for the root which has 2 degrees of freedom

    MatrixXf jacobian(numRows, numCols);
    // set root columns (first 2 columns)
    for (int i=0; i<numRows; ++i) {
        if (this->isXRow(i)) {
            jacobian(i, 0) = epsilon;
            jacobian(i, 1) = 0;
        } else {
            jacobian(i, 0) = 0;
            jacobian(i, 1) = epsilon;
        }
    }

    for (int i=0; i<allJoints.size(); ++i) {
        if (i == 0) { continue; } // skip root, its already set
        int col = i+1;

        for (int row=0; row<numRows; row=row+2) {
            int effectedIndex = row/2;
            Joint2D* effected = lockedJoints[effectedIndex];

            // root case
            if (this->isRoot(*effected)) {
                jacobian(row, col) = 0; // no one can effect the root
                jacobian(row+1, col) = 0;
                continue;
            }

            // other cases
            int effectorIndex = i;
            Joint2D* effector = allJoints[effectedIndex];

            // effected not a child of effector
            if (!this->canEffect(*effector, *effected)) {
                jacobian(row, col) = 0;
                jacobian(row+1, col) = 0;
                continue;
            }

            // the effector will have an effect, must handle to get values
            Vector3f effectedMath = qtToEigenMath(*posLockedJoints[effectedIndex]);
            Vector3f effectorMath = qtToEigenMath(posAllJoints[this->getParentIndex(allJoints, effectorIndex)]); // we want the pos of what we are roating around
            Vector3f delChange = Vector3f::UnitZ().cross(effectedMath - effectorMath);
            jacobian(row, col) = delChange.x();
            jacobian(row+1, col) = delChange.y();
        }
    }

    return jacobian;
}

std::vector<int> A2Solution::getChildIndexes(std::vector<Joint2D*>& allJoints, int currIndex) {
    std::vector<int> indexes;
    Joint2D* curr = allJoints[currIndex];
    int numChildren = curr->get_children().size();

    if (numChildren == 0) { return indexes; } // returns empty list if no child

    int found = 0;
    for(int i=currIndex+1; i<allJoints.size(); ++i) {
        if (allJoints[i]->get_parents()[0] == curr) {
            indexes.push_back(i);
            ++found;
            if (found == numChildren) { break; }
        }
    }

    return indexes;
}

int A2Solution::getParentIndex(std::vector<Joint2D*>& allJoints, int currIndex) {
    Joint2D* curr = allJoints[currIndex];
    Joint2D* parent = curr->get_parents()[0];

    for(int i=currIndex-1; i>=0; --i) {
        if (allJoints[i] == parent) { return i; }
    }

    return -1; // This is an error scenario
}

bool A2Solution::canEffect(Joint2D& effector, Joint2D& effected) {
    Joint2D* j = &effected;
    if (&effector == &effected) { return true; } // node will obviously be effected when it is rotated

    bool hasParent = !j->get_parents().empty();
    while (hasParent) {
        Joint2D* parent = j->get_parents()[0];
        if (parent == &effector) {
            return true;
        } else {
            j = parent;
            hasParent = !j->get_parents().empty();
        }
    }
    return false;
}


bool A2Solution::isXRow(int rowIndex) {
    return (rowIndex % 2 == 0);
}

void A2Solution::setRelevantJoints(Joint2D* selected) {
    // add root first always (assumes root is set)
    this->m_used_joints.push_back(this->m_root);
    this->pos_used_joints.push_back(this->m_root->get_position());

    if (m_root->is_locked()) {
        this->m_locked_joints.push_back(this->m_root);
        this->pos_locked_joints.push_back(&this->pos_used_joints.back());

        if (m_root == selected) { this->m_selected_index = 0; }
    }

    Joint2D* current;
    std::deque<Joint2D*> queue;
    for (Joint2D* child : this->m_root->get_children()) {
        queue.push_back(child);
    }

    while (queue.size() > 0) {
        current = queue.front();
        for (Joint2D* child : current->get_children()) {
            queue.push_back(child);
        }

        this->m_used_joints.push_back(current);
        this->pos_used_joints.push_back(current->get_position());

        if (current->is_locked()) {
            this->m_locked_joints.push_back(current);
            this->pos_locked_joints.push_back(&this->pos_used_joints.back());

            if (current == selected) { this->m_selected_index = this->m_used_joints.size() - 1; }
        }
        queue.pop_front();
    }
}

void A2Solution::setRoot() {
    for (Joint2D* joint : this->m_joints) {
        if (this->isRoot(*joint)) {
            this->m_root = joint;
            return;
        }
    }
}

void A2Solution::doFkPass(Joint2D& joint, QVector2D mouse_pos) {
    if (this->isRoot(joint)) {
        // When root is chosen, we should translate it and it's children
        QVector2D change = mouse_pos - joint.get_position();

        this->doMov = true;
        this->movToMake = change;
//        this->moveJointBy(joint, change);

    } else {
        // When non-root chosen, we rotate the selected the nodes and its children
        Joint2D* parent = joint.get_parents()[0];
        QVector2D parentPos = parent->get_position();
        QVector2D mathVecToJoint = this->qtToMathCoords(joint.get_position() - parentPos);
        QVector2D mathVecToMouse = this->qtToMathCoords(mouse_pos - parentPos);
        float theta = this->angleToRotate(mathVecToJoint, mathVecToMouse);

        this->doRot = true;
        this->rotToMake = theta;
//        this->rotateJointBy(joint, mathVecToJoint, theta);
    }
}

void A2Solution::commitFk(Joint2D& joint) {
    if (this->doMov) {
        this->moveJointBy(joint, this->movToMake);
    }

    if (this->doRot) {
        QVector2D parentPos = joint.get_parents()[0]->get_position();
        QVector2D mathVecToJoint = this->qtToMathCoords(joint.get_position() - parentPos);
        this->rotateJointBy(joint, mathVecToJoint, this->rotToMake);
    }

    this->doMov = false;
    this->doRot = false;
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

Vector3f A2Solution::qtToEigenMath(QVector2D qtVec) {
    QVector2D mathVec = qtVec;
    mathVec.setY(-qtVec.y());
    return Vector3f(mathVec.x(), mathVec.y(), 0);
}

QVector2D A2Solution::eigenMathToQt(Vector3f mathVec) {
    QVector2D qtVec = QVector2D(mathVec.x(), mathVec.y());
    qtVec.setY(-mathVec.y());
    return qtVec;
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
