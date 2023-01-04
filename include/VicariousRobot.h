#ifndef VICARIOUS_ROBOT_H
#define VICARIOUS_ROBOT_H

#include <map>
#include "Eigen/Dense"
#include "Eigen/Core"
#include "RBMotion/Kine.h"
#include "RBMotion/RobotInterface.h"

#include "Vicarious.h"
#include "VicariousUtils.h"

class VicariousRobot : public RobotInterface {
public:
    Kine externalArm;
    Kine cameraArm;
    Kine leftArm;
    Kine rightArm;
    Kine leftGrip;
    Kine rightGrip;

    double midDist;
    double armLink1offset;

    Eigen::Matrix4d elbowPlane;
    VicariousRobot() = default;
    explicit VicariousRobot(std::map<std::string, std::shared_ptr<RBLink>> RBMap);
    void transformEffectorFramesToView(double angle, Eigen::Matrix4d& effLeft, Eigen::Matrix4d& effRight);
};


#endif //VICARIOUS_ROBOT_H
