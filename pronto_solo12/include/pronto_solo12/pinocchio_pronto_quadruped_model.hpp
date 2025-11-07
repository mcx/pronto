#pragma once

#include <unordered_map>

// Pinocchio FWD Declarations need to be included first:
#include <pinocchio/fwd.hpp>

// Pronto
#include <pronto_quadruped_commons/feet_contact_forces.h>
#include <pronto_quadruped_commons/feet_jacobians.h>
#include <pronto_quadruped_commons/forward_kinematics.h>


// Pinocchio
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace pronto
{
namespace quadruped
{
class PinocchioProntoQuadrupedModel : public pronto::quadruped::ForwardKinematics, public pronto::quadruped::FeetContactForces, public pronto::quadruped::FeetJacobians
{
public:
    typedef pronto::quadruped::JointState JointState;
    typedef pronto::quadruped::LegID LegID;
    typedef pronto::quadruped::Vector3d Vector3d;
    typedef pronto::quadruped::Matrix3d Matrix3d;
    typedef pronto::quadruped::FootJac FootJac;

public:
    PinocchioProntoQuadrupedModel()
    {};
    PinocchioProntoQuadrupedModel(pinocchio::Model& model, const std::string& base_link_name, const std::string& lf_name, const std::string& rf_name, const std::string& lh_name, const std::string& rh_name);
    ~PinocchioProntoQuadrupedModel() override {}

    // Methods for ForwardKinematics
    Vector3d getFootPos(const JointState& q, const LegID& leg) final;
    Matrix3d getFootOrientation(const JointState& q, const LegID& leg) final;
    inline LegVectorMap getFeetPos(const JointState& q) final;

    using pronto::quadruped::FeetContactForces::getFootGRF;

    // Methods for FeetContactForces
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
                    const Vector3d& omegad = Vector3d::Zero()) final;
    // bool getFeetGRF(const JointState& q,
    //                 const JointState& qd,
    //                 const JointState& tau,
    //                 const Quaterniond& orient,
    //                 LegVectorMap& feet_grf,
    //                 const JointState& qdd = JointState::Zero(),
    //                 const Vector3d& xd = Vector3d::Zero(),
    //                 const Vector3d& xdd = Vector3d::Zero(),
    //                 const Vector3d& omega = Vector3d::Zero(),
    //                 const Vector3d& omegad = Vector3d::Zero()) override;

    // Methods for FootJacobians
    FootJac getFootJacobian(const JointState& q, const LegID& leg) override;

    FootJac getFootJacobianAngular(const pronto::quadruped::JointState& /*q*/,
                                   const pronto::quadruped::LegID& /*leg*/) override
    {
        return FootJac::Identity();
    }

protected:
    pinocchio::Model model_;
    pinocchio::Data data_;

    // Pinocchio requires LF, RF, LH, RH as the joint order. The Pinocchio parser frequently returns LF, LH, RF, RH
    bool need_to_swap_RF_and_LH_;

    Eigen::VectorXd q_;
    Eigen::VectorXd v_;
    Eigen::VectorXd qdd_;
    Eigen::VectorXd tau_;
    Eigen::Matrix<double, 6, -1> jacobian_;
    Eigen::Matrix3d foot_jacobian_;
    Eigen::Vector3d tau_leg_;
    Eigen::Vector3d tau_id_leg_;

    std::size_t base_link_frame_id_;
    std::size_t lf_frame_id_;
    std::size_t rf_frame_id_;
    std::size_t lh_frame_id_;
    std::size_t rh_frame_id_;
    std::unordered_map<LegID, std::size_t> limb_frame_idx_;
    std::unordered_map<LegID, int> limb_order_map_;  ///< Limb order 0-3

    LegVectorMap feet_pos_ = LegVectorMap(Vector3d::Zero());  // for getFeetPos

    void findFrameIdOrThrow(const std::string& frame_name, std::size_t& frame_id);
};
}  // namespace quadruped
}  // namespace pronto
