#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/publisher.hpp"
#include "pronto_tuning/spline.h"
#include <vector>
#include <array>
#include <string>
#include <cmath>
#include <iostream>
#include <cassert>
#include <future>
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "pi3hat_moteus_int_msgs/msg/omni_mulinex_command.hpp"
#include "pi3hat_moteus_int_msgs/msg/joints_states.hpp"

#define PUB_TO 15

using CmdMsg = pi3hat_moteus_int_msgs::msg::OmniMulinexCommand;
using StopSrv = std_srvs::srv::Empty;
using ContrSrv = std_srvs::srv::SetBool;
using ServiceResponseFuture = rclcpp::Client<ContrSrv>::SharedFuture;
namespace command_node
{
    class CommandNode : public rclcpp::Node
    {
        public:
            CommandNode():
            Node("command_node"),
            srv_req_(std::make_shared<ContrSrv::Request>()),
            srv_res_(std::make_shared<ContrSrv::Response>())
            {
                rclcpp::QoS cmd_qos(10);
                rclcpp::ServicesQoS srvs_qos = rclcpp::ServicesQoS();
                rclcpp::PublisherOptions opt_cmd;
                node_cb_grp_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                this->declare_parameter("controller_name",std::string());
                this->declare_parameter("topic_name",std::string());
                this->declare_parameter("timer_period_ms",100);
                period_ = get_parameter("timer_period_ms").as_int();
                topic_name_ = this->get_parameter("topic_name").as_string();
                controller_name_ = this->get_parameter("controller_name").as_string();
                this->declare_parameter("set_point_list",std::vector<std::string>());
                if(!this->get_parameter("set_point_list",spline_list_))
                {
                    RCLCPP_ERROR(get_logger(), "Error during the parsing of spline list");
                    throw(std::logic_error("yaml file error"));
                }
                if(spline_list_.empty())
                {
                    RCLCPP_ERROR(get_logger(), "Spline list can not be empty");
                    throw(std::logic_error(""));
                }
                for(auto &spline:spline_list_)
                {

                    this->declare_parameter(spline + ".vx",std::nan(""));
                    this->declare_parameter(spline + ".vy",std::nan(""));
                    this->declare_parameter(spline + ".omega",std::nan(""));
                    this->declare_parameter(spline + ".time",std::nan(""));
                }

                // init QOS structure

                cmd_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

                std::chrono::milliseconds dur_qos =std::chrono::milliseconds(period_ + 5);
                cmd_qos.deadline(dur_qos);
                opt_cmd.event_callbacks.deadline_callback =
                [this](rclcpp::QOSDeadlineOfferedInfo & event)
                {
                    RCLCPP_WARN(this->get_logger(),"not respect pub time");
                };

                spline_vx_list_.resize(spline_list_.size());
                spline_vy_list_.resize(spline_list_.size());
                spline_om_list_.resize(spline_list_.size());
                spline_time_list_.resize(spline_list_.size());
                for(size_t i = 0; i < spline_list_.size(); i++)
                {
                    spline_vx_list_[i] = get_parameter(spline_list_[i] + ".vx").as_double();
                    spline_vy_list_[i] = get_parameter(spline_list_[i] + ".vy").as_double();
                    spline_om_list_[i] = get_parameter(spline_list_[i] + ".omega").as_double();
                    spline_time_list_[i] = get_parameter(spline_list_[i] + ".time").as_double();
                    if( spline_vx_list_[i] == std::nan("") ||
                        spline_vy_list_[i] == std::nan("") ||
                        spline_om_list_[i] == std::nan("") ||
                        spline_time_list_[i] == std::nan(""))
                    {
                        RCLCPP_ERROR(get_logger(), "Error during the spline param parsing at index %ld",i);
                        assert(true);
                    }
                    RCLCPP_INFO(get_logger(),"the param of spline %ld are %f,%f,%f,%f", i, spline_vx_list_[i], spline_vy_list_[i], spline_om_list_[i], spline_time_list_[i]);
                    if(i!= 0)
                    {
                        if(spline_time_list_[i] <= spline_time_list_[i-1])
                        {
                            RCLCPP_ERROR(get_logger(), "The Time must be monotonic, time param error at index at index %ld",i);
                            assert(true);
                        }
                    }
                }
                //
                vx_spline_.set_boundary(tk::spline::first_deriv,0.0,tk::spline::first_deriv,0.0);
                vx_spline_.set_points(spline_time_list_,spline_vx_list_,tk::spline::linear);
                vy_spline_.set_boundary(tk::spline::first_deriv,0.0,tk::spline::first_deriv,0.0);
                vy_spline_.set_points(spline_time_list_,spline_vy_list_,tk::spline::linear);
                om_spline_.set_boundary(tk::spline::first_deriv,0.0,tk::spline::first_deriv,0.0);
                om_spline_.set_points(spline_time_list_,spline_om_list_,tk::spline::linear);

                // vx_spline_ = std::make_unique<tk::spline>(spline_vx_list_,spline_time_list_,tk::spline::cspline);
                // vx_spline_(spline_time_list_,spline_vx_list_,tk::spline::cspline_hermite,false,tk::spline::first_deriv,0.0,tk::spline::first_deriv,0.0);
                // vy_spline_ = tk::spline(spline_time_list_,spline_vx_list_,tk::spline::cspline_hermite,false,tk::spline::first_deriv,0.0,tk::spline::first_deriv,0.0);
                // om_spline_ = tk::spline(spline_time_list_,spline_vx_list_,tk::spline::cspline_hermite,false,tk::spline::first_deriv,0.0,tk::spline::first_deriv,0.0);
                pub_ = create_publisher<CmdMsg>(controller_name_  + topic_name_,cmd_qos,opt_cmd);

                stop_client_ = create_client<StopSrv>("stop_record",srvs_qos.get_rmw_qos_profile());
                start_control_ = create_client<ContrSrv>(controller_name_ + "homing_srv",srvs_qos.get_rmw_qos_profile());
                stop_control_ = create_client<ContrSrv>(controller_name_ + "emergency_srv",srvs_qos.get_rmw_qos_profile());
                RCLCPP_INFO(this->get_logger(),"%d",start_control_->service_is_ready());
                // activate robot cotroller

                timer_ = create_wall_timer(std::chrono::duration<int,std::milli>(1000), std::bind(&CommandNode::init_callback,this),node_cb_grp_);



                //timer_ = create_wall_timer(std::chrono::duration<int,std::milli>(period_), std::bind(&CommandNode::timer_callback,this),node_cb_grp_);
                // std::vector<double> prova = {0.0,1.0,2.0,3.0,4.0,5.0,6.0};
                // for(size_t i = 0; i < prova.size(); i++)
                // {
                //     RCLCPP_INFO(get_logger(),"the value of spline computed in %f is %f",prova[i], vx_spline_(prova[i]));
                // }
                 RCLCPP_INFO(get_logger(),"complete init");
            }
        private:

            void req_resp(rclcpp::Client<StopSrv>::SharedFuture fut)
            {
                auto state = fut.wait_for(std::chrono::duration<int,std::milli>(500));
                if(state == std::future_status::ready)
                    RCLCPP_INFO(get_logger(),"Stop Record Request Completed");
            }
            void timer_callback()
            {
                rclcpp::Time time_stamp = this->get_clock()->now();
                double secs;
                if(offset_timer_ == 0.0)
                {
                    offset_timer_ = time_stamp.seconds();
                }
                // RCLCPP_INFO(get_logger(),"%f",time_stamp);
                secs = time_stamp.seconds() - offset_timer_ + spline_time_list_[0];

                if(secs <= spline_time_list_[spline_list_.size()-1])
                {
                    msg_.set__v_x(vx_spline_(secs));
                    msg_.set__v_y(vy_spline_(secs));
                    msg_.set__omega(om_spline_(secs));
                    msg_.set__height_rate(0.0);
                    msg_.header.set__stamp(time_stamp);
                //  RCLCPP_INFO(get_logger(),"now is %f and stop at %f and ms is [%f,%f,%f]",secs,spline_time_list_[spline_list_.size()-1],msg_.v_x,msg_.v_y,msg_.omega);
                    pub_->publish(msg_);
                }
                else
                {
                    RCLCPP_INFO(get_logger(),"The timer has been cancelled, stop sending reference");


                    timer_->cancel();
                    timer_ = create_wall_timer(std::chrono::duration<int,std::milli>(period_), std::bind(&CommandNode::stop_callback,this),node_cb_grp_);


                }
            }
            void init_callback()
            {
                if(start_control_->service_is_ready())
                {
                    srv_req_->data = true;
                    auto response_received_callback = [this](ServiceResponseFuture future) {
                    auto result = future.get();
                    RCLCPP_INFO(this->get_logger(), "Result is: %s with message %s", result->success?std::string("True").c_str():std::string("False").c_str(),

                    result->message.c_str());
                    if(result->success)
                    {
                        timer_->cancel();
                        timer_ = create_wall_timer(std::chrono::duration<int,std::milli>(period_), std::bind(&CommandNode::timer_callback,this),node_cb_grp_);
                    }
                    };
                    auto res_f = start_control_->async_send_request(srv_req_,response_received_callback);
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "no controller response");
                }


            }

            void stop_callback()
            {
                if(stop_control_->service_is_ready())
                {
                    auto req = std::make_shared<StopSrv::Request>();
                    srv_req_->data = true;
                    auto response_received_callback = [this](ServiceResponseFuture future) {
                        auto result = future.get();
                        RCLCPP_INFO(this->get_logger(), "Result is: %s with message %s", result->success?std::string("True").c_str():std::string("False").c_str(),
                        result->message.c_str());
                        if(result->success)
                        {
                            timer_->cancel();
                        }
                    };
                    auto res_f = stop_control_->async_send_request(srv_req_,response_received_callback);
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "no controller response");
                }


            }

            std::vector<std::string> spline_list_;
            std::string topic_name_;
            std::string controller_name_;
            std::vector<double> spline_vx_list_,spline_vy_list_,spline_om_list_,spline_time_list_;
            // std::unique_ptr<tk::spline> vx_spline_,vy_spline,omega_spline_;
            tk::spline vx_spline_,vy_spline_,om_spline_;
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<CmdMsg>::SharedPtr pub_;
            CmdMsg msg_;
            double offset_timer_ = 0.0;
            int period_;
            rclcpp::Client<StopSrv>::SharedPtr stop_client_;
            rclcpp::Client<ContrSrv>::SharedPtr  start_control_, stop_control_;

            std::shared_ptr<rclcpp::CallbackGroup> node_cb_grp_;
            std::shared_ptr<ContrSrv::Request> srv_req_;
            std::shared_ptr<ContrSrv::Response> srv_res_;

    };
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<command_node::CommandNode>());

    rclcpp::shutdown();

    return 0;
}
