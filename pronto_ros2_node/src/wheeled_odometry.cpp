#include "pronto_ros2_node/wheeled_odometry.hpp"

namespace pronto{

    WheeledOdometry::WheeledOdometry()
    {
    };
    WheeledOdometry::WheeledOdometry(rclcpp::Node::SharedPtr node) :
    node_ptr_(node)
    {
        int mode_param;
        //declare parameters
        node_ptr_->declare_parameter<double>(sensor_name + "r_xyz",0.0);
        node_ptr_->declare_parameter<double>(sensor_name + "r_chi",0.0);
        node_ptr_->declare_parameter<int>(sensor_name + "mode",0);
        //get parameters
        if(! node_ptr_->get_parameter(sensor_name + "r_xyz",r_lin_))
        {
                RCLCPP_ERROR(node_ptr_->get_logger(),"error in parsing r_xyz parameter, the default name [%f] will be use",DEFAULT_R_LIN);
                r_lin_ = DEFAULT_R_LIN;
        }
        if(! node_ptr_->get_parameter(sensor_name + "r_chi",r_ang_))
        {
                RCLCPP_ERROR(node_ptr_->get_logger(),"error in parsing r_chi parameter, the default name [%f] will be use",DEFAULT_R_ANG);
                r_ang_ = DEFAULT_R_ANG;
        }
        if(! node_ptr_->get_parameter(sensor_name + "mode",mode_param))
        {
                RCLCPP_ERROR(node_ptr_->get_logger(),"error in parsing mode parameter, the default name [%d] will be use",(int)DEFAULT_MODE);
                mode_ = OdomMode::MODE_BOTH;
        }
        else
            mode_ = static_cast<OdomMode>(mode_param);


        switch (mode_)
        {
        case OdomMode::MODE_LINEAR :
            z_indices_ = RBIS::velocityInds();
            z_meas_.resize(3);
            cov_odom_.resize(3,3);
            cov_odom_.setZero();
            cov_odom_ = std::pow(r_lin_,2)*Eigen::Matrix3d::Identity();
            break;
        case OdomMode::MODE_ANGULAR :
            z_indices_ = RBIS::angularVelocityInds();
            z_meas_.resize(3);
            cov_odom_.resize(3,3);
            cov_odom_.setZero();
            cov_odom_ = std::pow(r_ang_*M_PI/180.0,2)*Eigen::Matrix3d::Identity();
            break;
        case OdomMode::MODE_BOTH :
            z_indices_.resize(6);
            z_meas_.resize(6);
            cov_odom_.resize(6,6);
            z_indices_.head<3>() = RBIS::velocityInds();
            z_indices_.tail<3>() = RBIS::angularVelocityInds();
            cov_odom_.setZero();
            cov_odom_.topLeftCorner<3, 3>() = std::pow(r_lin_, 2) * Eigen::Matrix3d::Identity();
            cov_odom_.bottomRightCorner<3, 3>() = std::pow((r_ang_*M_PI/180.0), 2) * Eigen::Matrix3d::Identity();
            break;
        default:
            break;

        }
        RCLCPP_ERROR_STREAM(node_ptr_->get_logger(),"the z_indexes are "<<z_indices_);
    };

    RBISUpdateInterface* WheeledOdometry::processMessage(const OdomMsg* msg,StateEstimator *est )
    {
        uint64_t utime = msg->header.stamp.sec * std::pow(10,6) + int(msg->header.stamp.nanosec /1000);

        switch (mode_)
        {
        case OdomMode::MODE_LINEAR :
            z_meas_ << msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z;
            return new RBISIndexedMeasurement(
                z_indices_,
                z_meas_,
                cov_odom_,
                RBISUpdateInterface::sensor_enum::wheels_odom,
                utime
            );
            break;
         case OdomMode::MODE_ANGULAR :
             z_meas_ << msg->twist.angular.x,msg->twist.angular.y,msg->twist.angular.z;
            return new RBISIndexedMeasurement(
                z_indices_,
                z_meas_,
                cov_odom_,
                RBISUpdateInterface::sensor_enum::wheels_odom,
                utime
            );
            break;
         case OdomMode::MODE_BOTH :
            z_meas_ << msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z,msg->twist.angular.x,msg->twist.angular.y,msg->twist.angular.z;
            return new RBISIndexedMeasurement(
                z_indices_,
                z_meas_,
                cov_odom_,
                RBISUpdateInterface::sensor_enum::wheels_odom,
                utime
            );
            break;
        default:
            return nullptr;
            break;
        }
    };
};
