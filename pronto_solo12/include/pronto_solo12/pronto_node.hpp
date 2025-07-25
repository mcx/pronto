#pragma once

#include <rclcpp/rclcpp.hpp>
#include "pronto_ros/ros_frontend.hpp"
#include "pronto_ros/ins_ros_handler.hpp"
#include "pronto_ros/pose_msg_ros_handler.hpp"
#include "pronto_ros/scan_matcher_ros_handler.hpp"

#include "pronto_quadruped_ros/conversions.hpp"
#include "pronto_quadruped_ros/stance_estimator_ros.hpp"
#include "pronto_quadruped_ros/leg_odometer_ros.hpp"
#include "pronto_quadruped_ros/bias_lock_handler_ros.hpp"
#include "pronto_quadruped_ros/legodo_handler_ros.hpp"

#include "pronto_solo12/feet_jacobians.hpp"
#include "pronto_solo12/feet_contact_forces.hpp"
// #include "pronto_solo12/dynamics.hpp"
#include "pronto_solo12/forward_kinematics.hpp"


// TODO: add visual odometry
namespace pronto {
namespace solo{

template <class MsgT>
struct is_dummy_msg {
  static const bool value = false;
};

template <>
struct is_dummy_msg<std_msgs::msg::Header> {
  static const bool value = true;
};

// use the std_msgs::msg::Header as a placeholder for a dummy message type
template <class JointStateMsgT, class ContactStateMsgT = std_msgs::msg::Header>
class ProntoNode : public rclcpp::Node {
public:
    using SensorList = std::vector<std::string>;
    using SensorSet = std::set<std::string>;

    ProntoNode(
        solo::ForwardKinematics& fwd_kin,
        solo::FeetJacobians& feet_jacs,
        solo::FeetContactForces& feet_forces);

    virtual ~ProntoNode(){};

    virtual void init(bool subscribe = true);
    virtual void run();

protected:

    // quadruped added here
    quadruped::StanceEstimatorROS stance_estimator;
    quadruped::LegOdometerROS leg_odometer;
    // quadruped::ImuBiasLockROS imu_bias_lock;

    // SensingModule<JointStateMsgT>& legodo_handler_;
    quadruped::LegodoHandlerROS legodo_handler_;
    quadruped::ImuBiasLockROS bias_lock_handler_;
    ROSFrontEnd front_end;
    SensorList init_sensors;
    SensorList active_sensors;
    SensorSet all_sensors; // sets have unique elements
    // pointers to the modules we might want to initialize
    std::shared_ptr<InsHandlerROS> ins_handler_;
    std::shared_ptr<PoseHandlerROS> pose_handler_;
    std::shared_ptr<ScanMatcherHandler> sm2_handler_;
};

template <class JointStateMsgT, class ContactStateMsgT>
ProntoNode<JointStateMsgT, ContactStateMsgT>::ProntoNode(
    solo::ForwardKinematics& fwd_kin,
    solo::FeetJacobians& feet_jacs,
    solo::FeetContactForces& feet_forces
) : Node("pronto_node"),
    stance_estimator(this->shared_from_this(), feet_forces),
    leg_odometer(this->shared_from_this(), feet_jacs, fwd_kin),
    /*imu_bias_lock(this->shared_from_this()),*/ legodo_handler_(this->shared_from_this(), stance_estimator, leg_odometer),
    bias_lock_handler_(this->shared_from_this()), front_end(this->shared_from_this()) {


    // get the list of active and init sensors from the param server
    if (!this->get_parameter("init_sensors", init_sensors)) {
        RCLCPP_ERROR(this->get_logger(), "Not able to get init_sensors param");
    }

    if (!this->get_parameter("active_sensors", active_sensors)) {
        RCLCPP_ERROR(this->get_logger(), "Not able to get active_sensors param");
    }

    bool publish_pose = false;
    if (!this->get_parameter("publish_pose", publish_pose)) {
        RCLCPP_WARN(this->get_logger(), "Not able to get publish_pose param. Not publishing pose.");
    }
}

template <class JointStateMsgT, class ContactStateMsgT>
void ProntoNode<JointStateMsgT, ContactStateMsgT>::init(bool subscribe) {
    // parameters:
    // is the module used for init?
    // do we need to move forward the filter once computed the update?
    // do we want to publish the filter state after the message is received?
    // which topic are we listening to for this module?
    bool init = false;
    bool active = false;
    bool roll_forward = false;
    bool publish_head = false;
    std::string topic;
    std::string secondary_topic;



    for (SensorList::iterator it = active_sensors.begin(); it != active_sensors.end(); ++it) {
        all_sensors.insert(*it);
    }
    for (SensorList::iterator it = init_sensors.begin(); it != init_sensors.end(); ++it) {
        all_sensors.insert(*it);
    }

    // now all_sensors contain a list which is the union (without repetition)
    // of the init sensors and active sensors
    // iterate over the sensors
    for (SensorSet::iterator it = all_sensors.begin(); it != all_sensors.end(); ++it) {

        //declare parameter
        declare_parameter<bool>(*it + ".roll_forward_on_receive",false);
        declare_parameter<bool>(*it + ".publish_head_on_message",false);
        declare_parameter<std::string>(*it + ".topic","");
        declare_parameter<std::string>(*it + ".secondary_topic","");

        if (!this->get_parameter(*it + ".roll_forward_on_receive", roll_forward)) {
            RCLCPP_WARN_STREAM(this->get_logger(),"Not adding sensor \"" << *it << "\".");
            RCLCPP_WARN(this->get_logger(), "Param \"roll_forward_on_receive\" not available.");
            continue;
        }
        if (!this->get_parameter(*it + ".publish_head_on_message", publish_head)) {
            RCLCPP_WARN_STREAM(this->get_logger(),"Not adding sensor \"" << *it << "\".");
            RCLCPP_WARN(this->get_logger(), "Param \"publish_head_on_message\" not available.");
            continue;
        }
        if (!this->get_parameter(*it + ".topic", topic)) {
            RCLCPP_WARN_STREAM(this->get_logger(),"Not adding sensor \"" << *it << "\".");
            RCLCPP_WARN(this->get_logger(), "Param \"topic\" not available.");
            continue;
        }
        // check if the sensor is also used to initialize
        init = (std::find(init_sensors.begin(), init_sensors.end(), *it) != init_sensors.end());
        active = (std::find(active_sensors.begin(), active_sensors.end(), *it) != active_sensors.end());

        // is the IMU module in the list? Typically yes.
        if (it->compare("ins") == 0) {
            ins_handler_ = std::make_shared<InsHandlerROS>(this->shared_from_this());
            if (active) {
                front_end.addSensingModule(*ins_handler_, *it, roll_forward, publish_head, topic, subscribe);
            }
            if (init) {
                front_end.addInitModule(*ins_handler_, *it, topic, subscribe);
            }
        }
        // is the leg odometry module in the list?
        // TODO the leg odometry handler object is constructed anyway, because
        // it is passed to the constructor as a reference. It should be
        // constructed only if we want to use it, but it is hard do handle this
        // from here.
        if(it->compare("legodo") == 0) {
            if(active){
                front_end.addSensingModule(legodo_handler_, *it, roll_forward, publish_head, topic, subscribe);
                // if secondary topic is provided, and the second message is not dummy,
                // attempt to cast the leg odometry handler as a DualHandler instead of SingleHandler
                if(!is_dummy_msg<ContactStateMsgT>::value && // non sono sicuro sia corretto
                   this->get_parameter(*it + "/secondary_topic", secondary_topic))
                {
                    try{
                        RCLCPP_INFO_STREAM(this->get_logger(),"Subscribing to secondary topic for legodo: " << secondary_topic);
                    front_end.addSecondarySensingModule(dynamic_cast<DualSensingModule<JointStateMsgT,ContactStateMsgT>&>(legodo_handler_),
                                                        *it,
                                                        secondary_topic,
                                                        subscribe);
                  } catch(const std::bad_cast & e){
                    RCLCPP_WARN_STREAM (this->get_logger(), "Could not use the provided Leg Odometry handler as DualSensingModule<"
                                    << type_name<JointStateMsgT>() << ", " << type_name<ContactStateMsgT>() << ">.");
                    RCLCPP_WARN(this->get_logger(),e.what());
                    }
                }
                else {
                    RCLCPP_WARN_STREAM(this->get_logger(), "Legodo not subscribing to secondary topic: Dummy message check: " << is_dummy_msg<ContactStateMsgT>::value);
                }
            }
            if(init){
                front_end.addInitModule(legodo_handler_, *it, topic, subscribe);
            }
        }
        if(it->compare("pose_meas") == 0){
            pose_handler_ = std::make_shared<PoseHandlerROS>(this->shared_from_this());
            if(active){
                front_end.addSensingModule(*pose_handler_, *it, roll_forward, publish_head, topic, subscribe);
            }
            if(init){
                front_end.addInitModule(*pose_handler_, *it, topic, subscribe);
            }
        }

        if(it->compare("bias_lock") == 0){
          if(!this->get_parameter(*it + "/secondary_topic", secondary_topic)){
              RCLCPP_WARN_STREAM(this->get_logger(),"Not adding sensor \"" << *it << "\".");
              RCLCPP_WARN_STREAM(this->get_logger(),"Param \"secondary_topic\" not available.");
              continue;
          }
          if(active){
            front_end.addSensingModule(bias_lock_handler_, *it, roll_forward, publish_head, topic, subscribe);
            front_end.addSecondarySensingModule(bias_lock_handler_, *it, secondary_topic, subscribe);
          }
        }

        // if(it->compare("scan_matcher") == 0 ){
        //   bool use_relative_pose = true;
        //   this->get_parameter(*it + "/relative_pose", use_relative_pose);
        //   RCLCPP_WARN_STREAM(this->get_logger(),"Scan matcher will use " << (use_relative_pose ? "relative " : "absolute ") << "pose");

        //   if(use_relative_pose){
        //     sm_handler_ = std::make_shared<LidarOdometryHandlerROS>(this->shared_from_this());
        //     if(active){
        //         front_end.addSensingModule(*sm_handler_, *it, roll_forward, publish_head, topic, subscribe);
        //     }
        //     if(init){
        //         front_end.addInitModule(*sm_handler_, *it, topic, subscribe);
        //     }
        //   } else {
        //     sm2_handler_ = std::make_shared<ScanMatcherHandler>(this->shared_from_this());
        //     if(active){
        //         front_end.addSensingModule(*sm2_handler_, *it, roll_forward, publish_head, topic, subscribe);
        //     }
        //     if(init){
        //         front_end.addInitModule(*sm2_handler_, *it, topic, subscribe);
        //     }
        //   }
        // }

        // TODO: add vicon and fovis
    }
}

template <class JointStateMsgT, class ContactStateMsgT>
void ProntoNode<JointStateMsgT, ContactStateMsgT>::run()
{
    init(true);
    rclcpp::spin(this->shared_from_this());
    rclcpp::shutdown();
}
}  // namespace solo
}  // namespace pronto
