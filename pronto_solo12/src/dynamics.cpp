#include "pronto_solo12/dynamics.hpp"

namespace pronto{
namespace solo{

    void Dynamics::computeInertiaMatrix(const JointState& q)
    {
        pinocchio::crba(model_, data_, q);

        // crba() computes only the upper triangular part of the matrix M, to obtain the full matrix we apply:
        data_.M.triangularView<Eigen::StrictlyLower>() = data_.M.transpose().triangularView<Eigen::StrictlyLower>();

        prev_q = q;
    }

    Eigen::Matrix3d Dynamics::getInertiaMatrix(const JointState& q, const LegID& leg)
    {
        if(q != prev_q){
            Dynamics::computeInertiaMatrix(q);
        }

        return data_.M.block<3, 3>(leg * 3, leg * 3);
    }

    void Dynamics::computeDynamics(const JointState& q, const JointState& qd){

        // TODO: check if it's possible to use rnea() for a faster calculation
        pinocchio::nonLinearEffects(model_, data_, q, qd);

        prev_q = q;
    }

    Dynamics::Vector3d Dynamics::getNonLinear(const JointState& q, const JointState& qd, const LegID& leg)
    {
        if(q != prev_q){ // suppose that if q didn't change, qd didn't change as well
            Dynamics::computeDynamics(q, qd);
        }

        return data_.nle.block<3, 1>(leg * 3, 0);
    }



} // namespace solo
} // namespace pronto
