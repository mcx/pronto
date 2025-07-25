#pragma once

#include <pronto_quadruped_commons/forward_kinematics.h>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include "pronto_solo12/feet_jacobians.hpp"


namespace pronto {
namespace solo{

class ForwardKinematics : public pronto::quadruped::ForwardKinematics {
public:

    typedef pronto::quadruped::JointState JointState;
    typedef pronto::quadruped::LegID LegID;
    typedef pronto::quadruped::Vector3d Vector3d;
    typedef pronto::quadruped::Matrix3d Matrix3d;

    ForwardKinematics(pronto::solo::FeetJacobians& feet_jacobian) :
    feet_jacobian_(feet_jacobian)
    {}

    Vector3d getFootPos(const JointState& q, const LegID& leg) override;
    Matrix3d getFootOrientation(const JointState& q, const LegID& leg) override;


private:

    pronto::solo::FeetJacobians feet_jacobian_;

};

}
}
