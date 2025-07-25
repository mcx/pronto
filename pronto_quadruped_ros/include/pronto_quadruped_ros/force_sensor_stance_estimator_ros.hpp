#pragma once

#include <pronto_quadruped/ForceSensorStanceEstimator.hpp>
#include <rclcpp/rclcpp.hpp>

namespace pronto {
namespace quadruped {

class ForceSensorStanceEstimatorROS : public ForceSensorStanceEstimator {
public:
  ForceSensorStanceEstimatorROS(double force_threshold = 50);
  ForceSensorStanceEstimatorROS(rclcpp::Node::SharedPtr nh);
  ~ForceSensorStanceEstimatorROS() override {}
};

}  // namespace quadruped
}  // namespace pronto
