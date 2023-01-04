#ifndef VICARIOUS_H
#define VICARIOUS_H

#include "RBMotion/Kine.h"

namespace Vicarious
{
    struct TransientData{
        Eigen::Vector3d sphere_center;
        Eigen::Vector3d lastElbowPos;
        Eigen::Vector3d candidateElbowPos;
        Eigen::Matrix4d wristT;
        Eigen::Vector3d wrist_pos;
        Eigen::Vector3d wrist_axis_z;
    };

    class CameraIKSolver : public IKSolverInterface{
        Eigen::Matrix4d cameraRefT;
    public:
        explicit CameraIKSolver(Kine &chain) : IKSolverInterface(chain) {
            cameraRefT = this->chain->treeMap["camera_link_4"]->absTransform;
        }
        std::vector<double> solve(Eigen::Matrix4d leftEff, Eigen::Matrix4d rightEff);
    };


    class ArmIKSolver : public IKSolverInterface{
        double upArmLength;
        double lowArmLength;
        double wristLinkLength;
        double d_angle = EIGEN_PI/1080.0;
//        Eigen::Vector3d lastWristPos;
        Eigen::Matrix4d effectorLocalTransform;
        std::shared_ptr<Kine> kine;
        std::shared_ptr<Eigen::Matrix4d> elbowPlane;

        double elbowIKSolve(const Eigen::Matrix4d& endEff, TransientData &data, double joint7);

    public:
        explicit ArmIKSolver(Kine &chain, Eigen::Matrix4d &elbowPlane) : IKSolverInterface(chain) {
//            kine = std::make_shared<Kine>(chain);
            this->elbowPlane = std::make_shared<Eigen::Matrix4d>(elbowPlane);

            auto upArmTransform = this->chain->relativeTransform("arm_link_5", "arm_link_3");
            auto lowArmTransform = this->chain->relativeTransform("arm_link_7", "arm_link_5");
            auto wristLinkTransform = this->chain->genericTreeMap["arm_link_8"]->localTransform;
            auto tmp = this->chain->genericTreeMap["arm_link_7"]->localTransform;
            upArmLength = upArmTransform(0, 3);
            lowArmLength = lowArmTransform(0, 3);
            wristLinkLength = wristLinkTransform(0,3);
            this->effectorLocalTransform = this->chain->genericTreeMap["arm_link_end_effector"]->localTransform;
        }

        std::vector<double> solve(Eigen::Matrix4d endEff) override;
    };

    class GripFKSolver : public FKSolverInterface{
    public:
        GripFKSolver(Kine &chain) : FKSolverInterface(chain){}
        void solve(std::vector<double> jawAngle) override;
    };
}

#endif //VICARIOUS_H
