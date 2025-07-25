#include <rclcpp/rclcpp.hpp>
#include <pronto_ros/ros_frontend.hpp>
#include <pronto_ros/ins_ros_handler.hpp>

using namespace MavStateEst;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::string prefix = "/state_estimator_pronto";
    rclcpp::NodeOptions options;
    options.set_arguments({"--ros-args", "-r", prefix});

    auto nh = rclcpp::Node::make_shared("pronto_ros_node", options);
    ROSFrontEnd front_end(nh);

    typedef std::vector<std::string> SensorList;
    SensorList init_sensors;
    SensorList active_sensors;

    if (!nh->get_parameter("init_sensors", init_sensors)) {
        RCLCPP_ERROR(nh->get_logger(), "Not able to get init_sensors param");
    }

    if (!nh->get_parameter("active_sensors", active_sensors)) {
        RCLCPP_ERROR(nh->get_logger(), "Not able to get active_sensors param");
    }

    bool publish_pose = false;
    if (!nh->get_parameter("publish_pose", publish_pose)) {
        RCLCPP_WARN(nh->get_logger(), "Not able to get publish_pose param. Not publishing pose.");
    }

    std::vector<std::string> param_names;
    nh->get_parameter_names(param_names);

    std::shared_ptr<InsHandlerROS> ins_handler_;
    // std::shared_ptr<LegOdometryHandlerROS> legodo_handler_;

    bool init = false;
    bool roll_forward = false;
    bool publish_head = false;
    std::string topic;

    for (SensorList::iterator it = active_sensors.begin(); it != active_sensors.end(); ++it) {
        if (!nh->get_parameter(*it + "/roll_forward_on_receive", roll_forward)) {
            RCLCPP_WARN_STREAM(nh->get_logger(), "Not adding sensor \"" << *it << "\".");
            RCLCPP_WARN_STREAM(nh->get_logger(), "Param \"roll_forward_on_receive\" not available.");
            continue;
        }

        if (!nh->get_parameter(*it + "/publish_head_on_message", publish_head)) {
            RCLCPP_WARN_STREAM(nh->get_logger(), "Not adding sensor \"" << *it << "\".");
            RCLCPP_WARN_STREAM(nh->get_logger(), "Param \"publish_head_on_message\" not available.");
            continue;
        }

        if (!nh->get_parameter(*it + "/topic", topic)) {
            RCLCPP_WARN_STREAM(nh->get_logger(), "Not adding sensor \"" << *it << "\".");
            RCLCPP_WARN_STREAM(nh->get_logger(), "Param \"topic\" not available.");
            continue;
        }

        // check if the sensor is also used to initialize
        if (std::find(init_sensors.begin(), init_sensors.end(), *it) != init_sensors.end()) {
            init = true;
        }

        if (it->compare("ins") == 0) {
            ins_handler_.reset(new InsHandlerROS(nh));
            front_end.addSensingModule(*ins_handler_, *it, init, roll_forward, publish_head, topic);
        }

        if (it->compare("legodo") == 0) {
            // Handle "legodo" case here if needed
        }
    }

    rclcpp::spin(nh);
    rclcpp::shutdown();

    return 0;
}
