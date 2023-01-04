#include "../include/VicariousRobot.h"

VicariousRobot::VicariousRobot(std::map<std::string, std::shared_ptr<RBLink>> RBMap) {
    this->RBMap = RBMap;

    Eigen::Matrix4d threeJSadjustTransform;
    threeJSadjustTransform << 1.0,  0.0, 0.0, 0.0,
                              0.0,  0.0, 1.0, 0.0,
                              0.0, -1.0, 0.0, 0.0,
                              0.0,  0.0, 0.0, 1.0;

    setAdjustTransform(threeJSadjustTransform);

    std::vector<std::string> prefixes = {"left_", "right_"};

    externalArm = Kine("External Arm");
    std::set<std::string> externalTreeEndConditions = {"left_arm_link_1", "right_arm_link_1", "camera_link_1"};
    externalArm.treeDiscovery(RBMap, "base_link_1", externalTreeEndConditions, prefixes);
    externalArm.fkSolver = std::make_shared<FKSolverInterface>(FKSolverInterface(externalArm));

    cameraArm = Kine("Camera");
    std::set<std::string> emptyEndConditions = {""};
    cameraArm.treeDiscovery(RBMap, "camera_link_1", emptyEndConditions, prefixes);
    cameraArm.fkSolver = std::make_shared<FKSolverInterface>(FKSolverInterface(cameraArm));

    std::map<std::string,double> armOffsets = {{"arm_link_2", EIGEN_PI}};
    leftArm = Kine("Left Arm");
    std::set<std::string> leftEndConditions = {"left_arm_link_8a", "left_arm_link_8b"};
    leftArm.treeDiscovery(RBMap, "left_arm_link_1", leftEndConditions, prefixes);
    leftArm.setJointsOffset(armOffsets);
    leftArm.fkSolver = std::make_shared<FKSolverInterface>(FKSolverInterface(leftArm));

    // Left Gripper
    leftGrip = Kine("Left Gripper");
    leftGrip.treeDiscovery(RBMap, "left_arm_link_7", emptyEndConditions, prefixes);
    leftGrip.fkSolver = std::make_shared<Vicarious::GripFKSolver>(Vicarious::GripFKSolver(leftGrip));

    // Right Arm
    rightArm = Kine("Right Arm");
    std::set<std::string> rightEndConditions = {"right_arm_link_8a", "right_arm_link_8b"};
    rightArm.treeDiscovery(RBMap, "right_arm_link_1", rightEndConditions, prefixes);
    rightArm.setJointsOffset(armOffsets);
    rightArm.fkSolver = std::make_shared<FKSolverInterface>(FKSolverInterface(rightArm));

    // Right Gripper
    rightGrip = Kine("Right Gripper");
    rightGrip.treeDiscovery(RBMap, "right_arm_link_7", emptyEndConditions, prefixes);
    rightGrip.fkSolver = std::make_shared<Vicarious::GripFKSolver>(Vicarious::GripFKSolver(rightGrip));

    std::vector<double> externalJoints = {0.5, EIGEN_PI/3.0, -2*EIGEN_PI/3.0, EIGEN_PI/6.0, -EIGEN_PI/8.0};
    externalArm.fk(externalJoints);

    std::vector<double> cameraJoints = {0.120, 0.0, 0.0, 0.0};
    cameraArm.fk(cameraJoints);

    // Elbow Plane Constraint Initialization
    auto Trot = Utils::eulXYZ2HMat(-EIGEN_PI/2.0, 0.0, 0.0);
    Eigen::Matrix4d Transl = Eigen::Matrix4d::Identity();
    Transl(2,3) = -0.02;
    Transl(0,3) = 0.120;
    elbowPlane = RBMap["base_link_5"]->absTransform * Trot * Transl;

//    std::cout << "This is elbow plane: " << std::endl;
//    std::cout << elbowPlane << std::endl;

//    std::vector<double> leftJoints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//    std::vector<double> leftJoints = {0.100, (3.0/8.0)*EIGEN_PI, 0.5*EIGEN_PI, EIGEN_PI/6.0, (3.5/5.0)*EIGEN_PI, EIGEN_PI/3.0, EIGEN_PI/12.0, 0.0};
//    std::vector<double> leftJoints = {0.120, (3.0/8.0)*EIGEN_PI, 0.55*EIGEN_PI, EIGEN_PI/6.0, (3.3/5.0)*EIGEN_PI, EIGEN_PI/3.0, EIGEN_PI/12.0, 0.0};
    std::vector<double> leftJoints = {0.120, (3.0/8.0)*EIGEN_PI, 0.50*EIGEN_PI, EIGEN_PI/12.0, (3.3/5.0)*EIGEN_PI, EIGEN_PI/3.0, EIGEN_PI/12.0, 0.0};
//    std::vector<double> leftJoints = {0.100, (3.0/8.0)*EIGEN_PI, EIGEN_PI/3.0, EIGEN_PI/6.0, (3.0/5.0)*EIGEN_PI, EIGEN_PI/3.0, EIGEN_PI/12.0, 0.0};
    leftArm.fk(leftJoints);

//    std::vector<double> rightJoints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//    std::vector<double> rightJoints = {0.100, -(3.0/8.0)*EIGEN_PI, 0.5*EIGEN_PI, -EIGEN_PI/6.0, (3.5/5.0)*EIGEN_PI, -EIGEN_PI/3.0, EIGEN_PI/12.0, 0.0};
//    std::vector<double> rightJoints = {0.120, -(3.0/8.0)*EIGEN_PI, 0.55*EIGEN_PI, -EIGEN_PI/6.0, (3.3/5.0)*EIGEN_PI, -EIGEN_PI/3.0, EIGEN_PI/12.0, 0.0};
    std::vector<double> rightJoints = {0.120, -(3.0/8.0)*EIGEN_PI, 0.50*EIGEN_PI, -EIGEN_PI/12.0, (3.3/5.0)*EIGEN_PI, -EIGEN_PI/3.0, EIGEN_PI/12.0, 0.0};
//    std::vector<double> rightJoints = {0.100, -(3.0/8.0)*EIGEN_PI, EIGEN_PI/3.0, -EIGEN_PI/6.0, (3.0/5.0)*EIGEN_PI, EIGEN_PI/3.0, EIGEN_PI/12.0, 0.0};
//    std::vector<double> rightJoints = {0.100, -(3.0/8.0)*EIGEN_PI, 0.5*EIGEN_PI, -EIGEN_PI/6.0, (3.5/5.0)*EIGEN_PI, -EIGEN_PI/3.0, EIGEN_PI/6.0, EIGEN_PI/6.0};
    rightArm.fk(rightJoints);

    vector<double> leftJawAngle = {M_PI_4};
    leftGrip.fk(leftJawAngle);

    vector<double> rightJawAngle = {M_PI_4};
    rightGrip.fk(rightJawAngle);

//    printKine(robot.externalArm);
//    printKine(robot.cameraArm);
//    printKine(robot.leftArm);
//    printKine(robot.rightArm);

    // IK

    leftArm.ikSolver = std::make_shared<Vicarious::ArmIKSolver>(Vicarious::ArmIKSolver(leftArm, elbowPlane));
    rightArm.ikSolver = std::make_shared<Vicarious::ArmIKSolver>(Vicarious::ArmIKSolver(rightArm, elbowPlane));
    cameraArm.ikSolver = std::make_shared<Vicarious::CameraIKSolver>(Vicarious::CameraIKSolver(cameraArm));

//    Eigen::Matrix4d left_target_transform = RBMap["left_arm_link_end_effector"]->absTransform;

//    std::cout << "Left target transform:" << std::endl;
//    std::cout << left_target_transform << std::endl;
//
//    Eigen::Matrix4d right_target_transform = RBMap["right_arm_link_end_effector"]->absTransform;
//    std::cout << "Right target transform:" << std::endl;
//    std::cout << right_target_transform << std::endl;
//
//    auto jointsLeft = leftArm.ik(left_target_transform);
//    auto jointsRight = rightArm.ik(right_target_transform);

//    std::cout << "Left arm joints: " << std::endl;
//    for(auto el : jointsLeft) std::cout << el << " ";
//    std::cout << std::endl;

//    std::cout << "Right arm joints: " << std::endl;
//    for(auto el : jointsRight) std::cout << el << " ";
//    std::cout << std::endl;

    auto childLinkTransform = this->RBMap["right_arm_link_1fixed"]->absTransform;
    auto baseLinkTransform = this->RBMap["base_link_5"]->absTransform;
    auto child2BaseTransform = baseLinkTransform.inverse() * childLinkTransform;
    midDist = child2BaseTransform(2,3);
    armLink1offset = child2BaseTransform(1,3);
}

void VicariousRobot::transformEffectorFramesToView(double angle, Eigen::Matrix4d& effLeft, Eigen::Matrix4d& effRight) {
    double armLink1TranslateOffset = tan(angle) * midDist;
    double neutralJ1pose = RBMap["camera_link_1"]->jState.pos;
    this->RBMap["left_arm_link_1"]->jState.pos = neutralJ1pose - armLink1TranslateOffset;
    this->RBMap["right_arm_link_1"]->jState.pos = neutralJ1pose + armLink1TranslateOffset;

    Eigen::Matrix4d translateToCameraCenter = Eigen::Matrix4d::Identity();
    translateToCameraCenter(0,3) = neutralJ1pose;
    translateToCameraCenter(2,3) = armLink1offset;

    Eigen::Matrix4d baseFrameForRotation = this->RBMap["base_link_5"]->absTransform * VicariousUtils::rotX(-M_PI_2) * translateToCameraCenter;
    effLeft = VicariousUtils::rotateFrameAroundFrameZ(effLeft, baseFrameForRotation, angle);
    effRight = VicariousUtils::rotateFrameAroundFrameZ(effRight, baseFrameForRotation, angle);
}