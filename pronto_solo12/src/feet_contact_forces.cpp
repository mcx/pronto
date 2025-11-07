#include <pronto_solo12/feet_contact_forces.hpp>
#include <pronto_quadruped_commons/geometry/rotations.h>
#include <pronto_quadruped_commons/joint_id_tricks.h>
#include "pronto_solo12/feet_contact_forces.hpp"

using namespace iit;  //rbd library

namespace pronto {
namespace solo{

    bool FeetContactForces::getFootGRF(const JointState &q,
                                        const JointState &qd,
                                        const JointState &tau,
                                        const Quaterniond &orient,
                                        const LegID &leg,
                                        Vector3d &foot_grf,
                                        const JointState &qdd,
                                        const Vector3d &xd,
                                        const Vector3d &xdd,
                                        const Vector3d &omega,
                                        const Vector3d &omegad) {
        Eigen::Matrix3d foot_jacobian = feet_jacs_.getFootJacobian(q, leg);

        rbd::Vector6D gravity_world = rbd::Vector6D::Zero();
        gravity_world(rbd::LZ) = -rbd::g;

        rbd::Vector6D gravity_base = rbd::Vector6D::Zero();
        rbd::Vector6D base_acceleration = rbd::Vector6D::Zero();
        rbd::VelocityVector base_twist = rbd::Vector6D::Zero();

        Eigen::Matrix3d R = pronto::commons::quatToRotMat(orient);

        gravity_base.segment(rbd::LX, 3) = R * gravity_world.segment(rbd::LX, 3);

        base_acceleration.segment(rbd::LX, 3) = xdd; //this is the absolute accel of the trunk without the gravity!!!
        base_acceleration.segment(rbd::AX, 3) = omegad;

        base_twist.segment(rbd::AX, 3) = omega;
        base_twist.segment(rbd::LX, 3) = xd;

        // rbd::ForceVector h_base;
        // JointState  h_joints;

        // From now on we use pinocchio
        M_leg = dynamics_.getInertiaMatrix(q, leg); // inertia matrix
        c_leg = dynamics_.getNonLinear(q, qd, leg); // C(q,qd)*qd + G(q)

        // 3 joints of the LF leg, on a 3x1 vector;
        Eigen::Vector3d tau_leg = quadruped::getLegJointState(LegID(leg), tau);
        Eigen::Vector3d qdd_leg = qdd.block<3, 1>(leg * 3, 0);

        // TODO: check if this computation has errors

        foot_grf = -(foot_jacobian.transpose()).inverse() *
                    (tau_leg - M_leg * qdd_leg - c_leg);

        return foot_grf.allFinite();
}
}  // namespace solo
}  // namespace pronto
