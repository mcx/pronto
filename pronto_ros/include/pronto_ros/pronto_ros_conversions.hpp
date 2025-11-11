#ifndef PRONTO_ROS_CONVERSIONS
#define PRONTO_ROS_CONVERSIONS
#endif

#include "pronto_core/definitions.hpp"
#include "pronto_msgs/msg/gps_data.hpp"
#include "pronto_msgs/msg/indexed_measurement.hpp"
#include "pronto_msgs/msg/filter_state.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "pronto_msgs/msg/visual_odometry_update.hpp"
#include "pronto_msgs/msg/lidar_odometry_update.hpp"
#include "pronto_msgs/msg/biped_force_torque_sensors.hpp"
#include "pronto_msgs/msg/joint_state_with_acceleration.hpp"
#include "pi3hat_moteus_int_msgs/msg/joints_states.hpp"

namespace pronto {

void gpsDataFromROS(const pronto_msgs::msg::GPSData& ros_msg,
                    GPSMeasurement& msg);

void indexMeasurementFromROS(const pronto_msgs::msg::IndexedMeasurement& ros_msg,
                             IndexedMeasurement& msg);

void filterStateFromROS(const pronto_msgs::msg::FilterState& ros_msg,
                        FilterState& msg);

void msgToImuMeasurement(const sensor_msgs::msg::Imu& imu_msg,
                         ImuMeasurement& imu_meas,
                         const int64_t &utime_offset = 0);

void poseMsgFromROS(const geometry_msgs::msg::PoseWithCovarianceStamped& msg,
                    PoseMeasurement& pose_meas);

void poseMsgFromROS(const geometry_msgs::msg::PoseStamped& msg,
                    PoseMeasurement& pose_meas);

void poseMeasurementFromROS(const nav_msgs::msg::Odometry& ros_msg,
                            PoseMeasurement& msg);

void rigidTransformFromROS(const geometry_msgs::msg::TransformStamped& msg,
                           RigidTransform& transf);

void jointStateFromROS(const sensor_msgs::msg::JointState& ros_msg,
                       JointState& msg);

void jointStateWithAccelerationFromROS(const pronto_msgs::msg::JointStateWithAcceleration& ros_msg,
                                       JointState& msg);

void visualOdometryFromROS(const pronto_msgs::msg::VisualOdometryUpdate& ros_msg,
                           VisualOdometryUpdate& msg);

void lidarOdometryFromROS(const pronto_msgs::msg::LidarOdometryUpdate& ros_msg,
                          LidarOdometryUpdate& msg);

void forceTorqueFromROS(const pronto_msgs::msg::BipedForceTorqueSensors& ros_msg,
                        pronto::ForceTorqueSensorArray& msg);

void JointStatesFromROS(const pi3hat_moteus_int_msgs::msg::JointsStates &ros_msg, JointState &msg);

inline void vectFromMsg(const geometry_msgs::msg::Vector3 & msg, Eigen::Vector3d & out)
{
  out.x() = msg.x;
  out.y() = msg.y;
  out.z() = msg.z;
}

inline void vectToMsg(const Eigen::Vector3d & in, geometry_msgs::msg::Vector3 & msg)
{
  msg.set__x(in(0));
  msg.set__y(in(1));
  msg.set__z(in(2));

}
inline void quatFromMsg(const geometry_msgs::msg::Quaternion & msg, Eigen::Quaterniond & out)
{
  out = Eigen::Quaterniond(msg.w, msg.x, msg.y, msg.z);
}

inline Eigen::Isometry3d transfToEigen(const geometry_msgs::msg::Transform & t)
{
  return Eigen::Isometry3d(
    Eigen::Translation3d(t.translation.x, t.translation.y, t.translation.z) *
    Eigen::Quaterniond(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z));
}
inline

geometry_msgs::msg::TransformStamped eigToTransf(const Eigen::Isometry3d & T)
{
  geometry_msgs::msg::TransformStamped t;
  t.transform.translation.x = T.translation().x();
  t.transform.translation.y = T.translation().y();
  t.transform.translation.z = T.translation().z();

  Eigen::Quaterniond q(T.rotation());
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  return t;
}
}  // namespace pronto
