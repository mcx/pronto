#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/publisher.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gazebo_msgs/msg/link_states.hpp"
#include <memory>
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/storage_options.hpp"
#define FB_LINK_SOLO "solo12::base_link"
#define FB_LINK_ANYM "anymal_c::base"
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "geometry_msgs/msg/vector3_stamped.hpp"

#include "std_srvs/srv/empty.hpp"
#include "pronto_msgs/msg/quadruped_stance.hpp"
#include "pronto_msgs/msg/quadruped_force_torque_sensors.hpp"
#define EXP_NAME "Estimator_Exp"
namespace tuning_node
{
    using std::placeholders::_1;
    using std::placeholders::_2;
    using Twist = geometry_msgs::msg::TwistStamped;
    using Pose = geometry_msgs::msg::PoseStamped;
    using LinkStates = gazebo_msgs::msg::LinkStates;
    using PosewCov = geometry_msgs::msg::PoseWithCovarianceStamped;
    using TwistwCov = geometry_msgs::msg::TwistWithCovarianceStamped;
    using StopSrv = std_srvs::srv::Empty;

    using Vec3 = geometry_msgs::msg::Vector3Stamped;

    using ForceEst = pronto_msgs::msg::QuadrupedForceTorqueSensors;
    using StanceEst = pronto_msgs::msg::QuadrupedStance;

    class Tuning_Node : public rclcpp::Node
    {
        public:
            Tuning_Node()
            : Node("tuning_node")
            {

                rclcpp::QoS out_qos(10);
                std::string exp_name;
                out_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
                this->declare_parameter("est_twist_topic", "");
                this->declare_parameter("est_pose_topic", "");
                this->declare_parameter("est_force","");
                this->declare_parameter("est_stance","");
                this->declare_parameter("odom_twist_cor", "");
                this->declare_parameter("folder_path", "");

                this->declare_parameter("exp_name","Pronto_Tuning");
                pub_ = this->create_publisher<Twist>("/tuning_node/ground_truth",rclcpp::QoS(10));
                sub_ = this->create_subscription<LinkStates>("gazebo/link_states",rclcpp::QoS(10),
                std::bind(&Tuning_Node::gz_sub,this,_1));
                try
                {
                    twist_est_t_name_ = this->get_parameter("est_twist_topic").as_string();
                    pose_est_t_name_ = this->get_parameter("est_pose_topic").as_string();
                    odom_twist_cor_t_name_ = this->get_parameter("odom_twist_cor").as_string();
                    bag_folder_path_ = this->get_parameter("folder_path").as_string();
                    exp_name =get_parameter("exp_name").as_string();
                    est_force_t_ = get_parameter("est_force").as_string();
                    est_stance_t_ = get_parameter("est_stance").as_string();
                }
                catch(const std::exception& e)
                {
                    RCLCPP_ERROR(get_logger(),"Error getting parameters: %s",e.what());
                    assert(true);
                }

                writer_ = std::make_unique<rosbag2_cpp::Writer>();

                time_t tm_now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

                auto lt_now = std::localtime(&tm_now);

                bag_folder_path_ = bag_folder_path_ + exp_name +std::string("_") + std::to_string(lt_now->tm_year+1900) + "_" + std::to_string(lt_now->tm_mon+1) + "_" + std::to_string(lt_now->tm_mday) + "_" +
            std::to_string(lt_now->tm_hour) + "_" + std::to_string(lt_now->tm_min) + "_" +std::to_string(lt_now->tm_sec);

                const rosbag2_storage::StorageOptions strg_opt({bag_folder_path_,"sqlite3"});
                try
                {
                    writer_->open(strg_opt);
                }
                catch(const std::exception& e)
                {
                    RCLCPP_ERROR(get_logger(),"Error opening bag: %s", e.what());
                    assert(true);
                }


                writer_->create_topic(
                    {twist_est_t_name_,
                    "geometry_msgs/msg/TwistWithCovarianceStamped",
                    rmw_get_serialization_format(),
                    ""}
                );

                writer_->create_topic(
                    {pose_est_t_name_,
                    "geometry_msgs/msg/PoseWithCovarianceStamped",
                    rmw_get_serialization_format(),
                    ""}
                );

                writer_->create_topic(
                    {odom_twist_cor_t_name_,

                    "geometry_msgs/msg/Vector3Stamped",

                    rmw_get_serialization_format(),
                    ""}
                );

                writer_->create_topic(
                    {est_force_t_,
                    "pronto_msgs/msg/QuadrupedForceTorqueSensors",
                    rmw_get_serialization_format(),
                    ""}
                );

                writer_->create_topic(
                    {est_stance_t_,
                    "pronto_msgs/msg/QuadrupedStance",
                    rmw_get_serialization_format(),
                    ""}
                );

                writer_->create_topic(
                    {"/tuning_node/ground_truth_twist",
                    "geometry_msgs/msg/TwistStamped",
                    rmw_get_serialization_format(),
                    ""}
                );

                writer_->create_topic(
                    {"/tuning_node/ground_truth_pose",
                    "geometry_msgs/msg/PoseStamped",
                    rmw_get_serialization_format(),
                    ""}
                );


                est_twist_sub_ = this->create_subscription<TwistwCov>(twist_est_t_name_,out_qos,
                std::bind(&Tuning_Node::est_twist_sub,this,_1));

                est_pose_sub_ = this->create_subscription<PosewCov>(pose_est_t_name_,out_qos,
                std::bind(&Tuning_Node::est_pose_sub,this,_1));

                odom_sub_ = this->create_subscription<Vec3>(odom_twist_cor_t_name_,rclcpp::QoS(10),
                std::bind(&Tuning_Node::odom_cor_sub,this,_1));

                est_stance_sub_ = create_subscription<StanceEst>(est_stance_t_,rclcpp::QoS(10),
                std::bind(&Tuning_Node::stance_sub,this,_1));

                est_force_sub_ = create_subscription<ForceEst>(est_force_t_,rclcpp::QoS(10),
                std::bind(&Tuning_Node::force_sub,this,_1));


                stop_srv_ = create_service<StopSrv>("stop_record",std::bind(&Tuning_Node::stop_record,this,_1,_2));

            }
            void est_twist_sub(std::shared_ptr<rclcpp::SerializedMessage> msg)
            {
                if(!stop)
                {
                    rclcpp::Time time_stamp = this->now();
                    const std::string name = twist_est_t_name_;
                    const std::string type = "geometry_msgs/msg/TwistWithCovarianceStamped";
                    writer_-> write(msg,name,type,time_stamp);
                }
                // RCLCPP_INFO(get_logger(),"PASS TWIST");
            }
            void est_pose_sub(std::shared_ptr<rclcpp::SerializedMessage> msg)
            {
                if(!stop)
                {
                    rclcpp::Time time_stamp = this->now();
                    const std::string name = pose_est_t_name_;
                    const std::string type = "geometry_msgs/msg/PoseWithCovarianceStamped";
                    writer_-> write(msg,name,type,time_stamp);
                }
                // RCLCPP_INFO(get_logger(),"PASS POSE");
            }
            void odom_cor_sub(std::shared_ptr<rclcpp::SerializedMessage> msg)
            {
                if(!stop)
                {
                    rclcpp::Time time_stamp = this->now();
                    const std::string name = odom_twist_cor_t_name_;

                    const std::string type = "geometry_msgs/msg/Vector3Stamped";

                    writer_-> write(msg,name,type,time_stamp);
                }
            }
            void stop_record(const std::shared_ptr<StopSrv::Request> req, const std::shared_ptr<StopSrv::Response> res)
            {
                RCLCPP_WARN(get_logger(),"Stop recording bag");
                stop = true;
                writer_->close();


            }

            void force_sub(std::shared_ptr<rclcpp::SerializedMessage> msg)
            {
                if(!stop)
                {
                    rclcpp::Time time_stamp = this->now();
                    const std::string name = est_force_t_;
                    const std::string type = "pronto_msgs/msg/QuadrupedForceTorqueSensors";
                    writer_-> write(msg,name,type,time_stamp);
                }
            }

            void stance_sub(std::shared_ptr<rclcpp::SerializedMessage> msg)
            {
                if(!stop)
                {
                    rclcpp::Time time_stamp = this->now();
                    const std::string name = est_stance_t_;
                    const std::string type = "pronto_msgs/msg/QuadrupedStance";
                    writer_-> write(msg,name,type,time_stamp);
                }
            }


            void gz_sub(const LinkStates& msg)
            {
                rclcpp::Time time_stamp = this->now();
                bool send = false;
                Twist base_twist_msg;
                Pose base_pose_msg;
                Eigen::Vector3d vel_vect,pose_vec;
                Eigen::Quaterniond w2b_or;
                if(!stop)
                {
                    // RCLCPP_INFO(this->get_logger(),"PASS");
                    for(size_t i = 0; i< msg.name.size(); i++)
                    {
                        if(msg.name[i].compare(FB_LINK_ANYM)== 0 && send==false)
                        {
                            vel_vect = Eigen::Vector3d(
                                msg.twist[i].linear.x,
                                msg.twist[i].linear.y,
                                msg.twist[i].linear.z);
                            w2b_or = Eigen::Quaterniond(
                                msg.pose[i].orientation.w,
                                msg.pose[i].orientation.x,
                                msg.pose[i].orientation.y,
                                msg.pose[i].orientation.z);
                            base_pose_msg.pose.set__position(msg.pose[i].position);
                            base_pose_msg.pose.set__orientation(msg.pose[i].orientation);
                            // vel_vect = w2b_or.toRotationMatrix().inverse()*vel_vect;

                            base_twist_msg.twist.linear.set__x(vel_vect(0));
                            base_twist_msg.twist.linear.set__y(vel_vect(1));
                            base_twist_msg.twist.linear.set__z(vel_vect(2));


                            vel_vect = Eigen::Vector3d(
                                msg.twist[i].angular.x,
                                msg.twist[i].angular.y,
                                msg.twist[i].angular.z);

                            // vel_vect = w2b_or.toRotationMatrix().inverse()*vel_vect;

                            base_twist_msg.twist.angular.set__x(vel_vect(0));
                            base_twist_msg.twist.angular.set__y(vel_vect(1));
                            base_twist_msg.twist.angular.set__z(vel_vect(2));
                            base_pose_msg.header.set__stamp(time_stamp);
                            base_twist_msg.header.set__stamp(time_stamp);

                            pub_->publish(base_twist_msg);

                            writer_->write(base_twist_msg,"/tuning_node/ground_truth_twist",time_stamp);
                            writer_->write(base_pose_msg,"/tuning_node/ground_truth_pose",time_stamp);



                            send = true;
                            // RCLCPP_INFO(this->get_logger(),"PASS");
                        }
                    }
                }
            }
        private:
            // TODO add writer to save the estimation data, the ground truth
            rclcpp::Publisher<Twist>::SharedPtr pub_;
            rclcpp::Subscription<LinkStates>::SharedPtr sub_;

            rclcpp::Subscription<Vec3>::SharedPtr odom_sub_;
            rclcpp::Subscription<TwistwCov>::SharedPtr est_twist_sub_;
            rclcpp::Subscription<PosewCov>::SharedPtr est_pose_sub_;
            rclcpp::Subscription<ForceEst>::SharedPtr est_force_sub_;
            rclcpp::Subscription<StanceEst>::SharedPtr est_stance_sub_;

            std::unique_ptr<rosbag2_cpp::Writer> writer_;
            std::string twist_est_t_name_;
            std::string pose_est_t_name_, est_force_t_,est_stance_t_;
            std::string odom_twist_cor_t_name_;
            std::string bag_folder_path_;
            rclcpp::Service<StopSrv>::SharedPtr stop_srv_;
            bool stop = false;
    };



}
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<tuning_node::Tuning_Node>());

    rclcpp::shutdown();
    return 0;
}
