#include "rclcpp/rclcpp.hpp"
#include <urdf/model.h>
#include <urdf/model.h>
// #include "pronto_ros/pronto_node.hpp"


#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>


#include <string>
#include <vector>
#include <array>


std::string urdf_file = "/home/jacopo/Documents/mul_pronto_ws/src/pronto_solo12/config/solo12.urdf";
pinocchio::Model robot_model;
const pinocchio::JointModelFreeFlyer root_joint;
const std::vector<std::string> jnt_id =
    {
        "LF_HAA",
        "LF_HFE",
        "LF_KFE",
        "RF_HAA",
        "RF_HFE",
        "RF_KFE",
        "LH_HAA",
        "LH_HFE",
        "LH_KFE",
        "RH_HAA",
        "RH_HFE",
        "RH_KFE"
    };
int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);
    // RCLCPP_INFO(rclcpp::get_logger("TEST"),"init");
    // pinocchio::urdf::buildModel(urdf_file, root_joint, robot_model);
    // RCLCPP_INFO(rclcpp::get_logger("TEST"),"made model");
    // std::vector<std::string> a = robot_model.names;

    // RCLCPP_INFO(rclcpp::get_logger("TEST"),"jnt name start here");
    // for(auto jnt:a)
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("TEST"),"jnt name is %s",jnt.c_str());
    // }
    // RCLCPP_INFO(rclcpp::get_logger("TEST"),"jnt name end here");

    // auto d = robot_model.joints;




    // for(auto jnt:d)
    // {
    //     jnt
    // }

    urdf::Model model;
    if(!model.initFile(urdf_file))
        RCLCPP_ERROR(rclcpp::get_logger("test"),"Error during the parsing of URDF");
    int jnt_num_ = model.joints_.size();

    std::vector<std::array<double,3>> hfe_list;
    std::vector<std::array<double,3>> kfe_list,haa_list;
    hfe_list.resize(4);
    kfe_list.resize(4);
    haa_list.resize(4);
    int kfe_count_ = 0;
    int hfe_count_ = 0;
    int haa_count_ = 0;
    for(auto jnt:model.joints_)
    {
        //  RCLCPP_INFO(rclcpp::get_logger("TEST"),"jnt name is %s",jnt.first.c_str());

        auto ax = jnt.second->axis;
        if(ax.x != 0.0 || ax.y != 0.0 || ax.z != 0.0)
        {

                RCLCPP_INFO(rclcpp::get_logger("TEST"),"jnt name is %s",jnt.first.c_str());

                if(jnt.first.find("HAA")!=std::string::npos)
                {
                    haa_list[haa_count_][0] = ax.x;
                    haa_list[haa_count_][1] = ax.y;
                    haa_list[haa_count_][2] = ax.z;
                    haa_count_++;
                }
                if(jnt.first.find("KFE")!=std::string::npos)
                {
                    kfe_list[kfe_count_][0] = ax.x;
                    kfe_list[kfe_count_][1] = ax.y;
                    kfe_list[kfe_count_][2] = ax.z;
                    kfe_count_++;
                }
                if(jnt.first.find("HFE")!=std::string::npos)
                {
                    hfe_list[hfe_count_][0] = ax.x;
                    hfe_list[hfe_count_][1] = ax.y;
                    hfe_list[hfe_count_][2] = ax.z;
                    hfe_count_++;
                }



        }

        // auto jnt_ax = jnt.second->axis;

        // RCLCPP_INFO(rclcpp::get_logger("TEST"),"jnt axis is [%f,%f,%f]",jnt.second->axis.x,jnt.second->axis.y,jnt.second->axis.z);
    }
    if(haa_count_ == 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("test")," the robot has no HAA joints, test if the fe has the same dimension and the joint axis is the same ");
        if(hfe_count_ == 4 && kfe_count_ == 4)
        {
            for(int i = 0; i < 4; i++)
            {
                if(hfe_list[i] == kfe_list[i])
                {
                    RCLCPP_INFO(rclcpp::get_logger("TEST"),"jnt %d is ok",i);
                }
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("TEST"),"jnt %d is not ok",i);
                }
            }
        }
        else
            RCLCPP_ERROR(rclcpp::get_logger("TEST"),"the urdf provided can not be used to the estimation pourpose");
    }
    else if(haa_count_ == 4 && hfe_count_ == 4 && kfe_count_ == 4)
    {
        for(int i = 0; i < 4; i++)
            {
                if(hfe_list[i] == kfe_list[i] && haa_list[i] != hfe_list[i])
                {
                    RCLCPP_INFO(rclcpp::get_logger("TEST"),"jnt %d is ok",i);
                }
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("TEST"),"jnt %d is not ok",i);
                }
            }
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("TEST"),"ERROR in urdf model");
    }
    RCLCPP_INFO(rclcpp::get_logger("TEST"),"jnt HAA axis are %d",haa_count_);
    RCLCPP_INFO(rclcpp::get_logger("TEST"),"jnt num are %d", jnt_num_);

    return 0;
}
