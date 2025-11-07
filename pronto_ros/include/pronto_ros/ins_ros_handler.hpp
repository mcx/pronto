#pragma once
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <pronto_core/ins_module.hpp>

namespace pronto{
    class InsHandlerROS : public SensingModule<sensor_msgs::msg::Imu>{
        public:
        InsHandlerROS(rclcpp::Node::SharedPtr nh);

        RBISUpdateInterface* processMessage(const sensor_msgs::msg::Imu *imu_msg, StateEstimator *est) override;

        bool processMessageInit(const sensor_msgs::msg::Imu *imu_msg,
                                const std::map<std::string, bool> &sensor_initialized,
                                const RBIS &default_state,
                                const RBIM &default_cov,
                                RBIS &init_state,
                                RBIM &init_cov) override;

    private:
        rclcpp::Node::SharedPtr nh_;
        ImuMeasurement imu_meas_;
        InsModule ins_module_;
        uint64_t counter = 0;
        uint16_t downsample_factor_ = 1;
        std::string imu_topic_ = "/sensors/imu";
        bool roll_forward_on_receive_ = true;
        int64_t utime_offset_ = 0;
    };
}  // namespace pronto
