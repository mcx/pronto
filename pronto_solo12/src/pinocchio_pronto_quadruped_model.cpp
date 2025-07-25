#include "pronto_solo12/pinocchio_pronto_quadruped_model.hpp"

#include <algorithm>
#include <chrono>

// Pinocchio
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>

namespace pronto
{
namespace quadruped
{
constexpr double kGravityConstant = 9.80665;

void SwapRFandLHInJointConfigurationVector(Eigen::Ref<JointState> q)
{
    assert(q.size() == 12);
    Eigen::Vector3d LH = q.segment<3>(3);
    Eigen::Vector3d RF = q.segment<3>(6);
    q.segment<3>(6) = LH;
    q.segment<3>(3) = RF;
}

void PinocchioProntoQuadrupedModel::findFrameIdOrThrow(const std::string& frame_name, std::size_t& frame_id)
{
    if (model_.existFrame(frame_name))
    {
        frame_id = model_.getFrameId(frame_name);
        std::cout << frame_name << " => " << frame_id << "\n";
    }
    else
    {
        throw std::invalid_argument("Link '" + frame_name + "' does not exist in the model.");
    }
}

PinocchioProntoQuadrupedModel::PinocchioProntoQuadrupedModel(pinocchio::Model& model, const std::string& base_link_name, const std::string& lf_name, const std::string& rf_name, const std::string& lh_name, const std::string& rh_name)
    : pronto::quadruped::ForwardKinematics(),
      pronto::quadruped::FeetContactForces(),
      pronto::quadruped::FeetJacobians(),
      model_(model),
      data_(model_),
      q_(pinocchio::neutral(model_)),
      v_(Eigen::VectorXd::Zero(model_.nv)),
      qdd_(Eigen::VectorXd::Zero(model_.nv)),
      tau_(Eigen::VectorXd::Zero(model_.nv)),
      jacobian_(6, model_.nv)
{
    // Find Pinocchio frame indices based on leg names:
    findFrameIdOrThrow(base_link_name, base_link_frame_id_);
    findFrameIdOrThrow(lf_name, lf_frame_id_);
    findFrameIdOrThrow(rf_name, rf_frame_id_);
    findFrameIdOrThrow(lh_name, lh_frame_id_);
    findFrameIdOrThrow(rh_name, rh_frame_id_);
    limb_frame_idx_[LegID::LF] = lf_frame_id_;
    limb_frame_idx_[LegID::RF] = rf_frame_id_;
    limb_frame_idx_[LegID::LH] = lh_frame_id_;
    limb_frame_idx_[LegID::RH] = rh_frame_id_;

    std::vector<std::size_t> limb_idx = {lf_frame_id_, rf_frame_id_, lh_frame_id_, rh_frame_id_};
    std::sort(limb_idx.begin(), limb_idx.end());
    limb_order_map_[LegID::LF] = std::distance(limb_idx.begin(), std::find(limb_idx.begin(), limb_idx.end(), lf_frame_id_));
    limb_order_map_[LegID::RF] = std::distance(limb_idx.begin(), std::find(limb_idx.begin(), limb_idx.end(), rf_frame_id_));
    limb_order_map_[LegID::LH] = std::distance(limb_idx.begin(), std::find(limb_idx.begin(), limb_idx.end(), lh_frame_id_));
    limb_order_map_[LegID::RH] = std::distance(limb_idx.begin(), std::find(limb_idx.begin(), limb_idx.end(), rh_frame_id_));

    // Determine if we need to swap the joint values for RF and LH before passing it to Pinocchio
    need_to_swap_RF_and_LH_ = (limb_order_map_[LegID::RF] > limb_order_map_[LegID::LH]);

    // for (int i = 0; i < model_.njoints; ++i)
    // {
    //     std::cout << "Joint #" << i << ": " << model_.names[i] << "\n";
    // }

    std::cout << " Limb ID: 'LF'=" << limb_order_map_[LegID::LF] << "\n";
    std::cout << " Limb ID: 'RF'=" << limb_order_map_[LegID::RF] << "\n";
    std::cout << " Limb ID: 'LH'=" << limb_order_map_[LegID::LH] << "\n";
    std::cout << " Limb ID: 'RH'=" << limb_order_map_[LegID::RH] << "\n";

    model_.gravity.linear().z() = -kGravityConstant;
}

Vector3d PinocchioProntoQuadrupedModel::getFootPos(const JointState& q, const LegID& leg)
{
    pinocchio::neutral(model_, q_);
    q_.tail<12>() = q;
    if (need_to_swap_RF_and_LH_) SwapRFandLHInJointConfigurationVector(q_.tail<12>());
    pinocchio::forwardKinematics(model_, data_, q_);
    pinocchio::updateFramePlacements(model_, data_);
    return data_.oMf[limb_frame_idx_[leg]].translation();
}

LegVectorMap PinocchioProntoQuadrupedModel::getFeetPos(const JointState& q)
{
    pinocchio::neutral(model_, q_);
    q_.tail<12>() = q;
    if (need_to_swap_RF_and_LH_) SwapRFandLHInJointConfigurationVector(q_.tail<12>());
    pinocchio::forwardKinematics(model_, data_, q_);
    pinocchio::updateFramePlacements(model_, data_);

    for (int i = 0; i < 4; ++i)
    {
        LegID leg = (LegID)i;
        feet_pos_[leg] = data_.oMf[limb_frame_idx_[leg]].translation();
    }

    return feet_pos_;
}

Matrix3d PinocchioProntoQuadrupedModel::getFootOrientation(const JointState& q, const LegID& leg)
{
    pinocchio::neutral(model_, q_);
    q_.tail<12>() = q;
    if (need_to_swap_RF_and_LH_) SwapRFandLHInJointConfigurationVector(q_.tail<12>());
    pinocchio::forwardKinematics(model_, data_, q_);
    pinocchio::updateFramePlacements(model_, data_);
    return data_.oMf[limb_frame_idx_[leg]].rotation();
}

//------------------- FEET CONTACT FORCES -------------------------------------

bool PinocchioProntoQuadrupedModel::getFootGRF(const JointState& q,
                                               const JointState& qd,
                                               const JointState& tau,
                                               const Quaterniond& orient,
                                               const LegID& leg,
                                               Vector3d& foot_grf,
                                               const JointState& qdd,
                                               const Vector3d& xd,
                                               const Vector3d& xdd,  // Absolute acceleration of the base without gravity
                                               const Vector3d& omega,
                                               const Vector3d& omegad)
{
    pinocchio::neutral(model_, q_);
    v_.setZero();
    q_.tail<12>() = q;
    v_.tail<12>() = qd;
    tau_.tail<12>() = tau;
    q_.segment<4>(3) = orient.coeffs();

    v_.head<3>() = xd;
    v_.segment<3>(3) = omega;

    // To make Pinocchio behave like Robcogen, set qdd to zero and then just xdd to the head:
    qdd_.setZero();
    qdd_.head<3>() = xdd;

    qdd_.segment<3>(3) = omegad;
    qdd_.segment<12>(6) = qdd;

    if (need_to_swap_RF_and_LH_)
    {
        SwapRFandLHInJointConfigurationVector(q_.tail<12>());
        SwapRFandLHInJointConfigurationVector(v_.tail<12>());
        SwapRFandLHInJointConfigurationVector(tau_.tail<12>());
        SwapRFandLHInJointConfigurationVector(qdd_.tail<12>());
    }

    // Normalize quaternion to avoid algorithm failure
    pinocchio::normalize(model_, q_);

    // Inverse dynamics
    pinocchio::rnea(model_, data_, q_, v_, qdd_);

    // Compute the frame Jacobian
    pinocchio::computeFrameJacobian(model_, data_, q_, limb_frame_idx_[leg], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_);
    foot_jacobian_ = jacobian_.block<3, 3>(0, 6 + 3 * limb_order_map_[leg]);

    // Extract the torques for the corresponding limb:
    tau_leg_ = tau_.block<3, 1>(6 + limb_order_map_[leg] * 3, 0);
    tau_id_leg_ = data_.tau.block<3, 1>(6 + limb_order_map_[leg] * 3, 0);
    foot_grf = -(foot_jacobian_.transpose()).inverse() * (tau_leg_ - tau_id_leg_);

    return foot_grf.allFinite();
}

// bool PinocchioProntoQuadrupedModel::getFeetGRF(const JointState& q,
//                                                const JointState& qd,
//                                                const JointState& tau,
//                                                const Quaterniond& orient,
//                                                LegVectorMap& feet_grf,
//                                                const JointState& qdd,
//                                                const Vector3d& xd,
//                                                const Vector3d& xdd,  // Absolute acceleration of the base without gravity
//                                                const Vector3d& omega,
//                                                const Vector3d& omegad)
// {
//     pinocchio::neutral(model_, q_);
//     v_.setZero();
//     q_.tail<12>() = q;
//     v_.tail<12>() = qd;
//     tau_.tail<12>() = tau;
//     q_.segment<4>(3) = orient.coeffs();

//     v_.head<3>() = xd;
//     v_.segment<3>(3) = omega;

//     // To make Pinocchio behave like Robcogen, set qdd to zero and then just xdd to the head:
//     qdd_.setZero();
//     qdd_.head<3>() = xdd;

//     qdd_.segment<3>(3) = omegad;
//     qdd_.segment<12>(6) = qdd;

//     if (need_to_swap_RF_and_LH_)
//     {
//         SwapRFandLHInJointConfigurationVector(q_.tail<12>());
//         SwapRFandLHInJointConfigurationVector(v_.tail<12>());
//         SwapRFandLHInJointConfigurationVector(tau_.tail<12>());
//         SwapRFandLHInJointConfigurationVector(qdd_.tail<12>());
//     }

//     // Normalize quaternion to avoid algorithm failure
//     pinocchio::normalize(model_, q_);

//     // Inverse dynamics
//     pinocchio::rnea(model_, data_, q_, v_, qdd_);

//     for (int i = 0; i < 4; ++i)
//     {
//         LegID leg = (LegID)i;
//         Vector3d& foot_grf = feet_grf[leg];

//         pinocchio::computeFrameJacobian(model_, data_, q_, limb_frame_idx_[leg], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_);
//         foot_jacobian_ = jacobian_.block<3, 3>(0, 6 + 3 * limb_order_map_[leg]);

//         // Extract the torques for the corresponding limb:
//         tau_leg_ = tau_.block<3, 1>(6 + limb_order_map_[leg] * 3, 0);
//         tau_id_leg_ = data_.tau.block<3, 1>(6 + limb_order_map_[leg] * 3, 0);
//         foot_grf = -(foot_jacobian_.transpose()).inverse() * (tau_leg_ - tau_id_leg_);
//     }

//     return feet_grf[LegID::LF].allFinite() && feet_grf[LegID::RF].allFinite() && feet_grf[LegID::LH].allFinite() && feet_grf[LegID::RH].allFinite();
// }

//------------------- FEET JACOBIANS ------------------------------------------
FootJac PinocchioProntoQuadrupedModel::getFootJacobian(const JointState& q,
                                                       const LegID& leg)
{
    pinocchio::neutral(model_, q_);
    q_.tail<12>() = q;
    if (need_to_swap_RF_and_LH_) SwapRFandLHInJointConfigurationVector(q_.tail<12>());
    pinocchio::forwardKinematics(model_, data_, q_);
    pinocchio::updateFramePlacements(model_, data_);
    pinocchio::computeFrameJacobian(model_, data_, q_, limb_frame_idx_[leg], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_);

    return jacobian_.block<3, 3>(0, 6 + 3 * limb_order_map_[leg]);
}

}  // namespace quadruped
}  // namespace pronto
