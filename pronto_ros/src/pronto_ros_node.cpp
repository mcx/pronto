#include <rclcpp/rclcpp.hpp>
#include "pronto_ros/ros_frontend.hpp"
#include "pronto_ros/ins_ros_handler.hpp"

using namespace pronto;


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // std::string prefix = "/state_estimator_pronto";
    // auto nh = rclcpp::Node::make_shared("pronto_ros_node" + std::string(prefix));
    auto nh = rclcpp::Node::make_shared("pronto_ros_node");

    auto front_end = std::make_shared<ROSFrontEnd>(nh);

    // get the list of active and init sensors from the param server
    typedef std::vector<std::string> SensorList;
    SensorList init_sensors;
    SensorList active_sensors;
    SensorList all_sensors;

    if(!nh->get_parameter("init_sensors", init_sensors)){
        RCLCPP_ERROR(nh->get_logger(), "Not able to get init_sensors param");
    }

    if(!nh->get_parameter("active_sensors", active_sensors)){
        RCLCPP_ERROR(nh->get_logger(), "Not able to get active_sensors param");
    }
    bool publish_pose = false;
    if(!nh->get_parameter("publish_pose", publish_pose)){
        RCLCPP_WARN(nh->get_logger(), "Not able to get publish_pose param. Not publishing pose.");
    }

    std::shared_ptr<InsHandlerROS> ins_handler_;
    // std::shared_ptr<ViconHandlerROS> vicon_handler_;

    bool init = false;
    bool active = false;
    bool roll_forward = false;
    bool publish_head = false;
    std::string topic;


    for(SensorList::iterator it = active_sensors.begin(); it != active_sensors.end(); ++it){
        all_sensors.push_back(*it);
    }
    for(SensorList::iterator it = init_sensors.begin(); it != init_sensors.end(); ++it){
        all_sensors.push_back(*it);
    }

    for(SensorList::iterator it = all_sensors.begin(); it != all_sensors.end(); ++it)
    {
        if(!nh->get_parameter(*it + "/roll_forward_on_receive", roll_forward)){
            RCLCPP_WARN(nh->get_logger(), "Not adding sensor \"%s \".",  (*it).c_str());
            RCLCPP_WARN(nh->get_logger(), "Param \"roll_forward_on_receive\" not available.");
            continue;
        }
        if(!nh->get_parameter(*it + "/publish_head_on_message", publish_head)){
            RCLCPP_WARN(nh->get_logger(), "Not adding sensor \"%s \".",  (*it).c_str());
            RCLCPP_WARN(nh->get_logger(), "Param \"publish_head_on_message\" not available.");
            continue;
        }
        if(!nh->get_parameter(*it + "/topic", topic)){
            RCLCPP_WARN(nh->get_logger(), "Not adding sensor \"%s \".",  (*it).c_str());
            RCLCPP_WARN(nh->get_logger(), "Param \"topic\" not available.");
            continue;
        }
        // check if the sensor is also used to initialize
        init = (std::find(init_sensors.begin(), init_sensors.end(), *it) != init_sensors.end());
        active = (std::find(active_sensors.begin(), active_sensors.end(), *it) != active_sensors.end());
        // is the IMU module in the list? Typically yes.
        if(it->compare("ins") == 0)
        {
            ins_handler_.reset();
            ins_handler_ = std::make_shared<InsHandlerROS>(nh);
            if(active){
                front_end->addSensingModule(*ins_handler_, *it, roll_forward, publish_head, topic);
            }
            if(init){
                front_end->addInitModule(*ins_handler_, *it, topic);
            }
        }
        // if(it->compare("vicon") == 0)
        // {
        //     vicon_handler_.reset();
        //     vicon_handler_ = std::make_shared<ViconHandlerROS>(nh);
        //     if(active){
        //         front_end->addSensingModule(*vicon_handler_, *it, roll_forward, publish_head, topic);
        //     }
        //     if(init){
        //         front_end->addInitModule(*vicon_handler_, *it, topic);
        //     }
        // }
    }

    rclcpp::shutdown();

    return 0;
}
