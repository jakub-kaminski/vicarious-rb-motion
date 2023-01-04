#include "VicariousUtils.h"

Eigen::Matrix4d VicariousUtils::rotX(double ang) {
    Eigen::Matrix4d res;
    res << 1.0, 0.0, 0.0, 0.0,
            0.0, cos(ang), -sin(ang), 0.0,
            0.0, sin(ang), cos(ang),  0.0,
            0.0, 0.0, 0.0, 1.0;
    return res;
}

Eigen::Matrix4d VicariousUtils::rotY(double ang) {
    Eigen::Matrix4d res;
    res << cos(ang), 0.0, sin(ang), 0.0,
            0.0, 1.0, 0.0, 0.0,
            -sin(ang), 0.0, cos(ang), 0.0,
            0.0, 0.0, 0.0, 1.0;
    return res;
}

Eigen::Matrix4d VicariousUtils::rotZ(double ang) {
    Eigen::Matrix4d res;
    res << cos(ang), -sin(ang), 0.0, 0.0,
            sin(ang), cos(ang), 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;
    return res;
}

Eigen::Matrix4d
VicariousUtils::rotateFrameAroundFrameZ(Eigen::MatrixXd frame, Eigen::Matrix4d baseFrame, double angle) {
    Eigen::Matrix4d result = baseFrame * rotZ(angle) * baseFrame.inverse() * frame;
    return result;
}

Eigen::Vector4d VicariousUtils::spiralPoint(double angle, double radius, double pitch) {
    double h = angle/(2.0*M_PI) * pitch;
    double x = cos(angle) * radius;
    double y = sin(angle) * radius;
    Eigen::Vector4d res = {x, y, h, 1.0};
    return res;
}

Eigen::MatrixXd VicariousUtils::spiralCompute() {
    double radius = 0.005;
    double pitch = 0.0030;
    double startAngle = M_PI_2;
    double incrAngle = M_PI_2;
    int scaleFactor = 2;
    int rotationPatches = 8;

    int loop1pts = 1 + rotationPatches * scaleFactor;
    int loop2pts = 2 * rotationPatches * scaleFactor - 1;
    int loop3pts = 1 + rotationPatches * scaleFactor;
    int numPts =  loop1pts + loop2pts + loop3pts;

    Eigen::MatrixXd pts(4, numPts); //points (x;y;z) for trajectory generation

    std::vector<double> angles;
    int colID = 0;
    for(int i = 0; i <= (rotationPatches * scaleFactor); ++i){
        double angleNow = startAngle + i * ( incrAngle / scaleFactor);
        Eigen::Vector4d pt = spiralPoint(angleNow, radius, pitch);
//      std::cout << "***" <<std::endl;
//      std::cout << pt << std::endl;
        pts.block(0,colID,4,1) = pt;
        ++colID;
    }

    double lastAngleLoop1 = startAngle + rotationPatches * incrAngle;
    Eigen::Vector4d lastPtLoop1 = spiralPoint(lastAngleLoop1, radius, pitch);

    for(int i = 1; i < (rotationPatches * 2 * scaleFactor); ++i){
        double angleNow = lastAngleLoop1 + i * ( incrAngle / scaleFactor);
        Eigen::Vector4d pt = spiralPoint(angleNow, radius, pitch);
        pt[2] = lastPtLoop1[2] - (pt[2] -lastPtLoop1[2]);
        pts.block(0,colID,4,1) = pt;
        ++colID;
    }

    double lastAngleLoop2 = lastAngleLoop1 + rotationPatches * incrAngle;
    Eigen::Vector4d lastPtLoop2 = pts.block(0,colID-1,4,1);

//    std::cout << lastPtLoop2 << std::endl;

    for(int i = 0; i <= ( rotationPatches * scaleFactor ); ++i){
        double angleNow = startAngle + i * ( incrAngle / scaleFactor);
        Eigen::Vector4d pt = spiralPoint(angleNow, radius, pitch);
        pt[2] += lastPtLoop2[2] - pitch*(1.5*incrAngle / (2.0 * M_PI));
//        std::cout << pt << std::endl;
        pts.block(0,colID,4,1) = pt;
        ++colID;
    }
//    std::cout << pts << std::endl;
    return pts;
}
