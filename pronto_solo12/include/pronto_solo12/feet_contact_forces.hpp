#pragma once

#include <pronto_quadruped_commons/feet_contact_forces.h>
#include <pronto_quadruped_commons/leg_vector_map.h>
#include "pronto_solo12/feet_jacobians.hpp"
#include "pronto_solo12/dynamics.hpp"

namespace pronto {
namespace solo{

class FeetContactForces : public pronto::quadruped::FeetContactForces {

public:
    using Vector3d = pronto::quadruped::Vector3d;
    using JointState = pronto::quadruped::JointState;
    using LegID = pronto::quadruped::LegID;
    using LegVectorMap = quadruped::LegVectorMap;

public:

    FeetContactForces(solo::FeetJacobians&  feet_jacs, solo::Dynamics& dynamics) :
        feet_jacs_(feet_jacs),
        dynamics_(dynamics)
    {}

    virtual ~FeetContactForces() override {}

    // Vector3d getFootGRF(const JointState& q,
    //                     const JointState& qd,
    //                     const JointState& tau,
    //                     const Quaterniond& orient,
    //                     const LegID& leg,
    //                     const JointState& qdd = JointState::Zero(),
    //                     const Vector3d& xd = Vector3d::Zero(),
    //                     const Vector3d& xdd = Vector3d::Zero(),
    //                     const Vector3d& omega = Vector3d::Zero(),
    //                     const Vector3d& omegad = Vector3d::Zero()) override {
    //     Vector3d res;
    //     getFootGRF(q, qd, tau, orient, leg, res, qdd, xd, xdd, omega, omegad);
    //     return res;
    // }

    bool getFootGRF(const JointState& q,
                    const JointState& qd,
                    const JointState& tau,
                    const Quaterniond& orient,
                    const LegID& leg,
                    Vector3d& foot_grf,
                    const JointState& qdd = JointState::Zero(),
                    const Vector3d& xd = Vector3d::Zero(),
                    const Vector3d& xdd = Vector3d::Zero(),
                    const Vector3d& omega = Vector3d::Zero(),
                    const Vector3d& omegad = Vector3d::Zero()) override;

    // bool getFeetGRF(const JointState& q,
    //                 const JointState& qd,
    //                 const JointState& tau,
    //                 const Quaterniond& orient,
    //                 LegVectorMap& feet_grf,
    //                 const JointState& qdd = JointState::Zero(),
    //                 const Vector3d& xd = Vector3d::Zero(),
    //                 const Vector3d& xdd = Vector3d::Zero(),
    //                 const Vector3d& omega = Vector3d::Zero(),
    //                 const Vector3d& omegad = Vector3d::Zero()) override {
    //     bool res_lf = getFootGRF(q, qd, tau, orient, LegID::LF, feet_grf[LegID::LF], qdd, xd, xdd, omega, omegad);
    //     bool res_rf = getFootGRF(q, qd, tau, orient, LegID::RF, feet_grf[LegID::RF], qdd, xd, xdd, omega, omegad);
    //     bool res_lh = getFootGRF(q, qd, tau, orient, LegID::LH, feet_grf[LegID::LH], qdd, xd, xdd, omega, omegad);
    //     bool res_rh = getFootGRF(q, qd, tau, orient, LegID::RH, feet_grf[LegID::RH], qdd, xd, xdd, omega, omegad);

    //     return(res_lf && res_rf && res_lh && res_rh);
    // }

    // LegVectorMap getFeetGRF(const JointState& q,
    //                         const JointState& qd,
    //                         const JointState& tau,
    //                         const Quaterniond& orient,
    //                         const JointState& qdd = JointState::Zero(),
    //                         const Vector3d& xd = Vector3d::Zero(),
    //                         const Vector3d& xdd = Vector3d::Zero(),
    //                         const Vector3d& omega = Vector3d::Zero(),
    //                         const Vector3d& omegad = Vector3d::Zero()) override {
    //     LegVectorMap res;
    //     getFeetGRF(q, qd, tau, orient, res, qdd, xd, xdd, omega, omegad);
    //     return res;
    // }

private:

    // Add here:
    // force transforms
    FeetJacobians feet_jacs_;
    Dynamics dynamics_;
    Eigen::Matrix3d M_leg;
    Eigen::Vector3d c_leg;

};
}
}
