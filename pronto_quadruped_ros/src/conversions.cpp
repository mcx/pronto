#include "pronto_quadruped_ros/conversions.hpp"
#include <rclcpp/rclcpp.hpp>

namespace pronto {
namespace quadruped {

bool jointStateFromROS(const sensor_msgs::msg::JointState& msg,
                       uint64_t& utime,
                       JointState& q,
                       JointState& qd,
                       JointState& qdd,
                       JointState& tau,
                       std::vector<std::string> jnt_names)
{
    // if the size of the joint state message does not match our own,
    // we silently return an invalid update
    std::size_t size = msg.position.size();

    // if (msg.position.size() != size ||
    //     msg.velocity.size() != size ||
    //     msg.effort.size() != size){
    //     RCLCPP_WARN(rclcpp::get_logger("jointStateFromROS"), "Joint State is expected %zu joints but %zu / %zu / %zu are provided.",
    //                      size, msg.position.size(), msg.velocity.size(), msg.effort.size());
    //     return false;
    // }
    // store message time in microseconds
    utime = msg.header.stamp.sec * std::pow(10,6) +msg.header.stamp.nanosec /1000;
    int j;
    for(int i=0; i<12; i++){
      j = -1;
      for(int k = 0; k<size; k++)
      {
        
        if(jnt_names[i].compare(msg.name[k])==0)
        {
          j = k;
        }
      }
      if(j==-1)
      {
        q(i) = 0.0;
        qd(i) = 0.0;
        tau(i) = 0.0;
      }
        // throw(std::logic_error("unconsistent joints nomenclature"));
      q(i) = msg.position[j];
      qd(i) = msg.velocity[j];
      tau(i) = msg.effort[j];
      // RCLCPP_INFO_STREAM(rclcpp::get_logger("JOINT CONV"),"jnts data "<< q <<" "<<qd <<" " << tau);
    }
    //q = Eigen::Map<const JointState>(msg.position.data());
    //qd = Eigen::Map<const JointState>(msg.velocity.data());
    //tau = Eigen::Map<const JointState>(msg.effort.data());

    qdd.setZero(); // TODO compute the acceleration

    return true;
}

bool jointStateWithAccelerationFromROS(const pronto_msgs::msg::JointStateWithAcceleration& msg,
                               uint64_t& utime,
                               JointState& q,
                               JointState& qd,
                               JointState& qdd,
                               JointState& tau,
                       std::vector<std::string> jnt_names)
{
    // if the size of the joint state message does not match our own,
    // we silently return an invalid update
    std::size_t size = static_cast<size_t>(q.size());

    if (msg.position.size() != size ||
        msg.velocity.size() != size ||
        msg.acceleration.size() != size ||
        msg.effort.size() != size){
        RCLCPP_WARN(rclcpp::get_logger("jointStateWithAccelerationFromROS"), "Joint State is expected %zu joints but %zu / %zu / %zu / %zu are provided.",
                         size, msg.position.size(), msg.velocity.size(), msg.acceleration.size(), msg.effort.size());
        return false;
    }
    // store message time in microseconds
    utime = msg.header.stamp.sec * std::pow(10,6) +msg.header.stamp.nanosec /1000;
    for(int i=0; i<12; i++){
      q(i) = msg.position[i];
      qd(i) = msg.velocity[i];
      qdd(i) = msg.acceleration[i];
      tau(i) = msg.effort[i];
    }

    return true;
}

bool JointsStatesFromROS(const pi3hat_moteus_int_msgs::msg::JointsStates& msg,
                               uint64_t& utime,
                               JointState& q,
                               JointState& qd,
                               JointState& qdd,
                               JointState& tau,
                       std::vector<std::string> jnt_names)
{
    // if the size of the joint state message does not match our own,
    // we silently return an invalid update
    std::size_t size = msg.position.size();

    // if (msg.position.size() != size ||
    //     msg.velocity.size() != size ||
    //     msg.effort.size() != size ||
    //     msg.temperature.size() != size){
    //     RCLCPP_WARN(rclcpp::get_logger("jointStateFromROS"), "Joint State is expected %zu joints but %zu / %zu / %zu / %zu are provided.",
    //                      size, msg.position.size(), msg.velocity.size(), msg.effort.size(), msg.temperature.size());
    //     return false;
    // }
    // store message time in microseconds
    utime = msg.header.stamp.sec * std::pow(10,6) +msg.header.stamp.nanosec /1000;
    int j;
    for(int i=0; i<12; i++){
      j = -1;
      for(int k = 0; k<size; k++)
      {
        
        if(jnt_names[i].compare(msg.name[k])==0)
        {
          j = k;
        }
      }
      if(j==-1)
      {
        q(i) = 0.0;
        qd(i) = 0.0;
        tau(i) = 0.0;
      }
        // throw(std::logic_error("unconsistent joints nomenclature"));
      q(i) = msg.position[j];
      qd(i) = msg.velocity[j];
      tau(i) = msg.effort[j];
      // RCLCPP_INFO_STREAM(rclcpp::get_logger("JOINT CONV"),"jnts data "<< q <<" "<<qd <<" " << tau);
    }
    //q = Eigen::Map<const JointState>(msg.position.data());
    //qd = Eigen::Map<const JointState>(msg.velocity.data());
    //tau = Eigen::Map<const JointState>(msg.effort.data());

    qdd.setZero(); // TODO compute the acceleration


    return true;
}


}  // namespace quadruped
}  // namespace pronto
