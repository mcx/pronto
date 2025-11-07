#pragma once

#include "pronto_solo12/feet_jacobians.hpp"

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace pronto{
namespace solo{

class Dynamics{
public:
    typedef pronto::quadruped::JointState JointState;
    typedef pronto::quadruped::LegID LegID;
    typedef pronto::quadruped::Vector3d Vector3d;

    Dynamics(pinocchio::Model & model,
                                pinocchio::Data & data) :
            model_(model), data_(data)
    {}

    virtual ~Dynamics() {};

    void computeDynamics(const JointState& q, const JointState& qd);
    void computeInertiaMatrix(const JointState& q);
    Eigen::Matrix3d getInertiaMatrix(const JointState& q, const LegID& leg);
    Vector3d getNonLinear(const JointState& q, const JointState& qd, const LegID& leg);


private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    JointState prev_q = JointState::Zero();

};

}  // namespace solo
}  // namespace pronto
