#include "pronto_solo12/forward_kinematics.hpp"

namespace pronto{
namespace solo{


    typedef pronto::quadruped::Vector3d Vector3d;
    typedef pronto::quadruped::Matrix3d Matrix3d;

    Vector3d ForwardKinematics::getFootPos(const JointState& q, const LegID& leg)
    {
        return feet_jacobian_.getFootPos(q, leg);
    }

    Matrix3d ForwardKinematics::getFootOrientation(const JointState& q, const LegID& leg)
    {
        return feet_jacobian_.getFootOrientation(q, leg);
    }
} // namespace solo

}