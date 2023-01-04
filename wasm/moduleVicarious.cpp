#include <iostream>
#include <chrono>
#include <unistd.h>


// Uncomment for EMSCRIPTEN ********
#include <emscripten.h>
#include <emscripten/bind.h>
#include <emscripten/val.h>

#include "RBMotion/RBTreeParser.h"
#include "RBMotion/Kine.h"
#include "RBMotion/Utils.h"

#include "../src/vicariousURDF.h"
#include "VicariousRobot.h"
#include "Trajectory/Trajectory.h"

#include "VicariousUtils.h"

Trajectory traj;

VicariousRobot robot;

Eigen::Matrix4d right_target_transform;
Eigen::Matrix4d left_target_transform;

auto startTimestamp = chrono::steady_clock::now();

int iter = 0;
int loops_done = 0;

double currentGlobalAngle = 0.0;
double currentJ1Angle = 0.120;

VicariousUtils::VelData VelGlobalRot(0.0, M_PI/180.0*0.25, M_PI/18.0, 0.0);
VicariousUtils::VelData VelGlobalJ1(0.120, 0.0004, 0.005, 0.0);

void setGlobalAngleGoal(double angle){
    auto nowTimestamp = chrono::steady_clock::now();
    double t = double(chrono::duration_cast<chrono::microseconds>(nowTimestamp - startTimestamp).count()) / 1000000.0;
    currentGlobalAngle = VelGlobalRot.updatePos(angle, t);
}

void setJ1AngleGoal(double dist){
    auto nowTimestamp = chrono::steady_clock::now();
    double t = double(chrono::duration_cast<chrono::microseconds>(nowTimestamp - startTimestamp).count()) / 1000000.0;
    currentJ1Angle = VelGlobalJ1.updatePos(dist, t);
}

void animationStep(){
    auto nowTimestamp = chrono::steady_clock::now();
    double t = double(chrono::duration_cast<chrono::microseconds>(nowTimestamp - startTimestamp).count()) / 1000000.0;
    while(t-8.0 > ((loops_done + 1) * traj.getTotalTime())){
        ++loops_done;
        traj.currentSegmentNumber = 0;
    }
    auto timeTransl = t - ( loops_done * traj.getTotalTime() );
    timeTransl -= 8.0;
    if(timeTransl < 0.0) timeTransl = 0.0;

    auto [x0, y0, z0] = traj.getStates(timeTransl);
    Eigen::Matrix4d effRight = right_target_transform;
    Eigen::Matrix4d effLeft = left_target_transform;

    effRight(0,3) += x0(0);
    effRight(1,3) += y0(0);
    effRight(2,3) += z0(0);

    Eigen::Matrix4d xrMatR = VicariousUtils::rotX(sin(t*0.8)*0.3 - 0.3);
    Eigen::Matrix4d yrMatR = VicariousUtils::rotY(sin(t*0.8)*0.3 + 0.2);
    effRight.block(0,0,3,3) = right_target_transform.block(0,0,3,3) * xrMatR.block(0,0,3,3) * yrMatR.block(0,0,3,3);

    effLeft(0,3) += x0(0);
    effLeft(1,3) += y0(0);
    effLeft(2,3) += z0(0);

    Eigen::Matrix4d xrMatL = VicariousUtils::rotX(sin(t*0.8)*0.3);
    Eigen::Matrix4d yrMatL = VicariousUtils::rotY(sin(t*0.8)*0.3 + 0.3);
    effLeft.block(0,0,3,3) = left_target_transform.block(0,0,3,3) * xrMatL.block(0,0,3,3) * yrMatL.block(0,0,3,3);

    robot.leftArm.fkSolver->solveLinksRange("arm_link_1", "arm_link_3", {currentJ1Angle, 0.0, 0.0});
    robot.rightArm.fkSolver->solveLinksRange("arm_link_1", "arm_link_3", {currentJ1Angle, 0.0, 0.0});

    robot.transformEffectorFramesToView(currentGlobalAngle, effLeft, effRight);

    // IK Solution
    auto jointsRight0 = robot.rightArm.ikSolver->solve(effRight);
    jointsRight0[1] -= EIGEN_PI;
    robot.rightArm.fk(jointsRight0);
    robot.rightGrip.fk({M_PI_4});

    auto jointsLeft0 = robot.leftArm.ikSolver->solve(effLeft);
    jointsLeft0[1] -= EIGEN_PI;
    robot.leftArm.fk(jointsLeft0);
    robot.leftGrip.fk({M_PI_4});

    auto lastCameraJoints = robot.cameraArm.ikSolver->solve(effLeft, effRight);
    std::vector<double> cameraJoints = {currentJ1Angle, 0.0, lastCameraJoints[0], lastCameraJoints[1]};
    robot.cameraArm.fk(cameraJoints);
}

std::vector<double> getTransformByID(std::string id){
    return robot.getTransformByID(id);
}

int _main() {
//int main() {
    std::shared_ptr<urdf::UrdfModel> model;
    model = urdf::UrdfModel::fromUrdfStr(std::string(vicarious));
    std::map<std::string, std::shared_ptr<RBLink>> RBMap = parseToRBLinkMap(model);

    robot = VicariousRobot(RBMap);
    right_target_transform= RBMap["right_arm_link_end_effector"]->absTransform;
    left_target_transform= RBMap["left_arm_link_end_effector"]->absTransform;

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block(0,0,3,3) = robot.RBMap["base_link_5"]->absTransform.block(0,0,3,3);

    Eigen::MatrixXd pts = VicariousUtils::spiralCompute();
    int numPts = pts.size() / 4;
    Eigen::MatrixXd ptsTransf = T * pts;
    Eigen::MatrixXd ptsReady(3, numPts);
    ptsReady = ptsTransf.block(0,0,3, numPts);
    double vel = 0.010;
    traj.computeMultiSegment(ptsReady, vel);

    animationStep();
    return 0;
}

// Uncomment for EMSCRIPTEN ********
EMSCRIPTEN_BINDINGS(module)
{
   emscripten::function("animationStep", &animationStep);
   emscripten::function("getTransformByID", &getTransformByID);
   emscripten::function("main", &_main);
   emscripten::function("setGlobalAngleGoal", &setGlobalAngleGoal);
   emscripten::function("setJ1AngleGoal", &setJ1AngleGoal);
   emscripten::register_vector<double>("vector<double>");
}


//std::map<std::string,std::vector<double>> fnGetMovingObjects(){
    //return robot.map;
//}

//std::vector<MyObject> fnGetMovingObjects(){
    //return robot.objectVec;
//}


//// 600 objects

//map 16 strings and 16 x 16 double


