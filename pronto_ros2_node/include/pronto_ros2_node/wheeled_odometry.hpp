#pragma once
#include <rclcpp/rclcpp.hpp>
#include <pronto_core/sensing_module.hpp>
#include "geometry_msgs/msg/twist_stamped.hpp"

#define DEFAULT_R_LIN 0.1
#define DEFAULT_R_ANG 0.1
#define DEFAULT_MODE 2
namespace pronto
{

    enum class OdomMode {MODE_LINEAR,
                      MODE_ANGULAR,
                      MODE_BOTH};



    using OdomMsg = geometry_msgs::msg::TwistStamped;
    class WheeledOdometry : public SensingModule<OdomMsg>
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        public:
            using MeasVector = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 6, 1>;
            using IndexVector = Eigen::Matrix<int, Eigen::Dynamic, 1, 0, 6, 1>;
            using CovMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, 6 ,6>;
        public:
            WheeledOdometry();
            WheeledOdometry(rclcpp::Node::SharedPtr node);

            ~WheeledOdometry()
            {};

            RBISUpdateInterface* processMessage(const OdomMsg *msg,
                                            StateEstimator *est) override;

            // this module do not produce any contribution to estimator initialization
            bool processMessageInit(const OdomMsg *msg,
                                const std::map<std::string, bool> &sensor_initialized,
                                const RBIS &default_state,
                                const RBIM &default_cov,
                                RBIS &init_state, RBIM &init_cov) override
            {
                return true;
            };
        private:

            OdomMode mode_;
            IndexVector z_indices_;
            MeasVector z_meas_;
            CovMatrix cov_odom_;
            rclcpp::Node::SharedPtr node_ptr_;
            const std::string sensor_name = "w_odom.";
            double r_lin_,r_ang_;
    };
}
