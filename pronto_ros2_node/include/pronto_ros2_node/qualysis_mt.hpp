#pragma once
#include "gazebo_msgs/msg/link_states.hpp"
#include "gazebo_msgs/msg/link_state.hpp"
#include "mocap4r2_msgs/msg/rigid_bodies.hpp"
#include "mocap4r2_msgs/msg/rigid_body.hpp"

#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "pronto_core/vicon_module.hpp"
#include "pronto_core/definitions.hpp"

namespace pronto
{
    class QualysisMTRosHandler : public SensingModule<mocap4r2_msgs::msg::RigidBodies>
    {
        public:
            QualysisMTRosHandler(rclcpp::Node::SharedPtr node);
            ~QualysisMTRosHandler(){};
            RBISUpdateInterface* processMessage(const mocap4r2_msgs::msg::RigidBodies* gz_ls_msg, StateEstimator *est) override;

            bool processMessageInit(const mocap4r2_msgs::msg::RigidBodies* gz_ls_msg,
                                const std::map<std::string, bool> &sensor_initialized,
                                const RBIS &default_state,
                                const RBIM &default_cov,
                                RBIS &init_state,
                                RBIM &init_cov) override;
            inline std::string get_sens_topic()
            {
                return mt_topic_;
            }

        private:
            bool get_base_link(mocap4r2_msgs::msg::RigidBodies msg,  mocap4r2_msgs::msg::RigidBody &body_pose)
            {

                for(size_t i=0; i < msg.rigidbodies.size(); i++)
                {
                    if(msg.rigidbodies[i].rigid_body_name.compare(rb_name_)== 0)
                    {
                        body_pose.set__pose(msg.rigidbodies[i].pose);
                        // if found exists only one pose in msg
                        return true;
                    }
                }
                return false;
            }

            rclcpp::Node::SharedPtr node_ptr_;
            ViconModule mt_modules_;
            std::string mt_topic_ = "";
            std::string rb_name_ = "";


    };
} // namespace pronto
