#include "pronto_ros2_node/qualysis_mt.hpp"


#define SENS_NAME "gazebo_mt."
#define DEFAULT_ROBOT "anymal_c"
#define DEFAULT_LINK "base"
#define DEFAULT_R_XYZ 0.001
#define DEFAULT_R_CHI 0.1
#define DEFAULT_TOPIC "gazebo/link_states"
namespace pronto
{
    QualysisMTRosHandler::QualysisMTRosHandler(rclcpp::Node::SharedPtr node):
    node_ptr_(node)
    {
        ViconConfig cfg;
        std::string robot_name,link_name,sensor_name = "qualysis_mt.";
        ViconMode default_mode = ViconMode::MODE_POSITION;
        int mode;
        // set the mt to robot rigid transformation as the identity
        cfg.body_to_vicon = pronto::Transform(Eigen::Isometry3d::Identity());

        node_ptr_->declare_parameter<std::string>(sensor_name + "robot_name","Omnicar");
        node_ptr_->declare_parameter<double>(sensor_name + "r_xyz",0.0);
        node_ptr_->declare_parameter<double>(sensor_name + "r_chi",0.0);
        node_ptr_->declare_parameter<int>(sensor_name + "mode",0);

        //get the parameters of the gazebo motion tracker module
        if(! node_ptr_->get_parameter(sensor_name + "robot_name",robot_name))
        {
                RCLCPP_ERROR(node_ptr_->get_logger(),"error in parsing robot_name parameter, the default name [%s] will be use",DEFAULT_ROBOT);
                robot_name = DEFAULT_ROBOT;
        }
        if(! node_ptr_->get_parameter(sensor_name + "r_xyz",cfg.r_vicon_xyz))
        {
                RCLCPP_ERROR(node_ptr_->get_logger(),"error in parsing base_link_name parameter, the default name [%f] will be use",DEFAULT_R_XYZ);
                cfg.r_vicon_xyz = DEFAULT_R_XYZ;
        }
        if(! node_ptr_->get_parameter(sensor_name + "r_chi",cfg.r_vicon_chi))
        {
                RCLCPP_ERROR(node_ptr_->get_logger(),"error in parsing base_link_name parameter, the default name [%f] will be use",DEFAULT_R_CHI);
                cfg.r_vicon_chi = DEFAULT_R_CHI;
        }
        if(! node_ptr_->get_parameter(sensor_name + "mode",mode))
        {
                RCLCPP_ERROR(node_ptr_->get_logger(),"error in parsing base_link_name parameter, the default name [%d] will be use",(int)default_mode);
                cfg.mode = default_mode;
        }
        else
            cfg.mode = static_cast<ViconMode>(mode);

        // if(! node_ptr_->get_parameter(sensor_name + "topic",mt_topic_))
        // {
        //         RCLCPP_ERROR(node_ptr_->get_logger(),"error in parsing base_link_name parameter, the default name [%s] will be use",DEFAULT_TOPIC);
        //         mt_topic_ = DEFAULT_TOPIC;
        // }

        rb_name_ = robot_name;

        mt_modules_ = ViconModule(cfg);

        RCLCPP_INFO(node_ptr_->get_logger(),"QualysisMTRosHandler Initialization compleated");

    };
    bool QualysisMTRosHandler::processMessageInit(const mocap4r2_msgs::msg::RigidBodies* mocap_msg,
                                const std::map<std::string, bool> &sensor_initialized,
                                const RBIS &default_state,
                                const RBIM &default_cov,
                                RBIS &init_state,
                                RBIM &init_cov)
    {
            mocap4r2_msgs::msg::RigidBody rb;
            pronto::RigidTransform mt_data;

            if(get_base_link(*mocap_msg,rb))
            {
                Eigen::Quaterniond rot(rb.pose.orientation.w,rb.pose.orientation.x,rb.pose.orientation.y,rb.pose.orientation.z);
                Eigen::Translation3d trasl(rb.pose.position.x,rb.pose.position.y,rb.pose.position.z);
                mt_data.transform = Eigen::Isometry3d(trasl * rot);
                mt_data.utime = mocap_msg->header.stamp.sec * std::pow(10,6) +mocap_msg->header.stamp.nanosec /1000;
                return mt_modules_.processMessageInit(&mt_data,sensor_initialized,default_state,default_cov,init_state,init_cov);
            }
            else
                return false;

    };
    RBISUpdateInterface* QualysisMTRosHandler::processMessage(const mocap4r2_msgs::msg::RigidBodies* mocap_msg, StateEstimator *est)
    {
        mocap4r2_msgs::msg::RigidBody rb;
        pronto::RigidTransform mt_data;
        // RCLCPP_INFO(controller_ptr_->get_logger()," start process message");
        if(get_base_link(*mocap_msg,rb))
        {
            Eigen::Quaterniond rot(rb.pose.orientation.w,rb.pose.orientation.x,rb.pose.orientation.y,rb.pose.orientation.z);
            Eigen::Translation3d trasl(rb.pose.position.x,rb.pose.position.y,rb.pose.position.z);
            mt_data.transform = Eigen::Isometry3d(trasl * rot);
            mt_data.utime = mocap_msg->header.stamp.sec * std::pow(10,6) +mocap_msg->header.stamp.nanosec /1000;
        //     RCLCPP_INFO(controller_ptr_->get_logger()," call vicon module Process message");

            return mt_modules_.processMessage(&mt_data,est);
        }
        else
            return nullptr;
    }
}
