#ifndef VICARIOUS_UTILS_H
#define VICARIOUS_UTILS_H

#include "Eigen/Dense"
#include <vector>

namespace VicariousUtils
{
   Eigen::Matrix4d rotX(double ang);
   Eigen::Matrix4d rotY(double ang);
   Eigen::Matrix4d rotZ(double ang);
   Eigen::Matrix4d rotateFrameAroundFrameZ(Eigen::MatrixXd frame, Eigen::Matrix4d baseFrame, double angle);

   Eigen::Vector4d spiralPoint(double angle, double radius, double pitch);
   Eigen::MatrixXd spiralCompute();

    struct VelData{
        double pos;
        double minError;
        double vel;
        double lastTime;
        VelData(double pos, double minError, double vel, double lastTime) : pos(pos), minError(minError), vel(vel), lastTime(lastTime){}

        double updatePos(double goal, double time){
            double error = goal - pos;
            if(abs(error) > minError){
                double deltaTime = time - lastTime;
                if(( goal - pos ) > 0.0) pos += vel * deltaTime;
                else pos -= vel * deltaTime;
            }
            lastTime = time;
            return pos;
        }
    };
}


#endif //VICARIOUS_UTILS_H
