#pragma once

#include <pronto_quadruped_commons/feet_jacobians.h>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace pronto {
namespace solo{

class FeetJacobians : public pronto::quadruped::FeetJacobians {
public:

    typedef pronto::quadruped::FootJac FootJac;
    typedef pronto::quadruped::JointState JointState;
    typedef pronto::quadruped::LegID LegID;
    typedef pronto::quadruped::Vector3d Vector3d;
    typedef pronto::quadruped::Matrix3d Matrix3d;

    FeetJacobians(pinocchio::Model & model,
                                pinocchio::Data & data) :
            model_(model), data_(data)
    {}

    virtual ~FeetJacobians() override {};

    void updateConfiguration(const JointState& q);

    pinocchio::Data::Matrix6x ComputeJacobian(const JointState& q, const LegID& leg);

    FootJac getFootJacobian(const JointState& q, const LegID& leg) override;
    FootJac getFootJacobianAngular(const JointState& q, const LegID& leg) override; // is this function used?

    Vector3d getFootPos(const JointState& q, const LegID& leg);
    Matrix3d getFootOrientation(const JointState& q, const LegID& leg);


private:

    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::FrameIndex leg_id;
    FootJac jacobian;
    pinocchio::Data::Matrix6x J;
    pinocchio::SE3 T;
    JointState prev_q = JointState::Zero();

};

}  // namespace solo
}  // namespace pronto

