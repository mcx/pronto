#include "pronto_ros/pose_msg_ros_handler.hpp"
#include "pronto_ros/pronto_ros_conversions.hpp"

namespace pronto {

PoseHandlerROS::PoseHandlerROS(rclcpp::Node::SharedPtr nh) : nh_(nh) {
    PoseMeasConfig cfg;
    std::string prefix = "pose_meas/";
    std::string mode_str;
    if (!nh_->get_parameter(prefix + "mode", mode_str)) {
        RCLCPP_WARN(nh_->get_logger(), "Parameter 'mode' not found. Setting to POSITION_ORIENT.");
        cfg.mode = PoseMeasMode::MODE_POSITION_ORIENT;
    } else {
        if (mode_str.compare("position_orient") == 0) {
            cfg.mode = PoseMeasMode::MODE_POSITION_ORIENT;
        } else {
            cfg.mode = PoseMeasMode::MODE_POSITION;
        }
    }
    if (!nh_->get_parameter(prefix + "no_corrections", cfg.number_of_corrections)) {
        RCLCPP_WARN(nh_->get_logger(), "Parameter 'no_corrections' not found. Setting to 100");
        cfg.number_of_corrections = 100;
    }

    if (!nh_->get_parameter(prefix + "r_xyz", cfg.r_pose_meas_xyz)) {
        RCLCPP_WARN(nh_->get_logger(), "Parameter 'r_xyz' not found. Setting to 0.01");
        cfg.r_pose_meas_xyz = 0.01;
    }
    if (!nh_->get_parameter(prefix + "r_chi", cfg.r_pose_meas_chi)) {
        RCLCPP_WARN(nh_->get_logger(), "Parameter 'r_chi' not found. Setting to 3.0");
        cfg.r_pose_meas_chi = 3.0;
    }

    pose_module_ = std::make_unique<PoseMeasModule>(cfg);
}

RBISUpdateInterface* PoseHandlerROS::processMessage(const geometry_msgs::msg::PoseWithCovarianceStamped *msg,
                                    StateEstimator *est)
{
    poseMsgFromROS(*msg, pose_meas_);
    return pose_module_->processMessage(&pose_meas_, est);
}

bool PoseHandlerROS::processMessageInit(const geometry_msgs::msg::PoseWithCovarianceStamped *msg,
                        const std::map<std::string, bool> &sensor_initialized,
                        const RBIS &default_state,
                        const RBIM &default_cov,
                        RBIS &init_state,
                        RBIM &init_cov)
{
    poseMsgFromROS(*msg, pose_meas_);
    return pose_module_->processMessageInit(&pose_meas_,
                                            sensor_initialized,
                                            default_state,
                                            default_cov,
                                            init_state,
                                            init_cov);
}
}
