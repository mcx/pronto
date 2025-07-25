#pragma once

#include <sensor_msgs/msg/joint_state.hpp>
#include <pronto_msgs/msg/joint_state_with_acceleration.hpp>
#include <pronto_quadruped_commons/declarations.h>
#include <pi3hat_moteus_int_msgs/msg/joints_states.hpp>

namespace pronto {
namespace quadruped {
bool jointStateFromROS(const sensor_msgs::msg::JointState& msg,
                       uint64_t& utime,
                       JointState& q,
                       JointState& qd,
                       JointState& qdd,
                       JointState& tau,
                        std::vector<std::string>  jnt_names=
                                    {"LF_HAA", "LF_HFE", "LF_KFE",
                                    "RF_HAA", "RF_HFE", "RF_KFE",
                                    "LH_HAA", "LH_HFE", "LH_KFE",
                                    "RH_HAA", "RH_HFE", "RH_KFE"});

bool jointStateWithAccelerationFromROS(const pronto_msgs::msg::JointStateWithAcceleration& msg,
                               uint64_t& utime,
                               JointState& q,
                               JointState& qd,
                               JointState& qdd,
                               JointState& tau,
                               std::vector<std::string>  jnt_names=
                                     {"LF_HAA", "LF_HFE", "LF_KFE",
                                      "RF_HAA", "RF_HFE", "RF_KFE",
                                      "LH_HAA", "LH_HFE", "LH_KFE",
                                      "RH_HAA", "RH_HFE", "RH_KFE"});

bool JointsStatesFromROS(const pi3hat_moteus_int_msgs::msg::JointsStates& msg,
                               uint64_t& utime,
                               JointState& q,
                               JointState& qd,
                               JointState& qdd,
                               JointState& tau,
                               std::vector<std::string>  jnt_names=
                                     {"LF_HAA", "LF_HFE", "LF_KFE",
                                      "RF_HAA", "RF_HFE", "RF_KFE",
                                      "LH_HAA", "LH_HFE", "LH_KFE",
                                      "RH_HAA", "RH_HFE", "RH_KFE"});



}  // namespace quadruped
}  // namespace pronto
