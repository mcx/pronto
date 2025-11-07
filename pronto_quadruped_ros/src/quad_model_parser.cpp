#include "pronto_quadruped_ros/quad_model_parser.hpp"
#include "rclcpp/rclcpp.hpp"
namespace pronto
{

    Model_Parser::Model_Parser(
        std::string urdf_path
    )
    {
        bool first;
        size_t ind;
        int leg_jnt,parse_leg_count,parse_jnt_leg_count;

        std::map<std::string,std::array<double,3>> leg_map;
        if(!model_.initString(urdf_path))
        {
            RCLCPP_INFO(rclcpp::get_logger("URDF_MODEL_PARSER"),"The Model parsing throw an error");
            urdf_usable_ = false;
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("URDF_MODEL_PARSER"),"The Model parsing  is performed corctly");
            urdf_usable_ = true;
            parse_leg_count = 0;
            for(ind = 0; ind < quadruped_leg_id.size(); ind ++)
            {
                parse_jnt_leg_count = 0;
                first = true;
                for(auto &model_jnt:model_.joints_)
                {
                    // RCLCPP_INFO(rclcpp::get_logger("URDF_Parser"),"the parse jnt is %s",model_jnt.first.c_str());
                    if(model_jnt.first.find(quadruped_leg_id[ind]) != std::string::npos)
                    {
                        // RCLCPP_INFO(rclcpp::get_logger("URDF_Parser"),"compare parse jnt is %s with leg id %s",model_jnt.first.c_str(),quadruped_leg_id[ind].c_str());
                        if(first)
                        {
                            parse_leg_count++;
                            first = false;
                        }
                        for(auto &jnt_leg_name:quadruped_jnt_id)
                        {
                            auto axis = model_jnt.second->axis;
                            if(model_jnt.first.find(jnt_leg_name) != std::string::npos)
                            {
                                if(axis.x != 0.0 || axis.y != 0.0 || axis.z != 0.0)
                                {
                                    parse_jnt_leg_count ++;
                                    leg_map.insert({jnt_leg_name,{axis.x,axis.y,axis.z}});
                                }
                            }
                        }
                    }
                }

                if(parse_jnt_leg_count == 2 || parse_jnt_leg_count == 3)
                {
                    if(!jnts_find_)
                        jnts_find_ = parse_jnt_leg_count;
                    else if(jnts_find_ != parse_jnt_leg_count)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("URDF_parser"),"All leg must have the same joints number per leg");
                        urdf_usable_=false;
                    }
                    kin_tree_.insert({quadruped_leg_id[ind],leg_map});
                    leg_map = std::map<std::string,std::array<double,3>>();
                }
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("URDF_parser"),"The leg %s has %d joints, its not correct",quadruped_leg_id[ind].c_str(),parse_jnt_leg_count);
                    urdf_usable_=false;
                }
            // jnt_names_.resize(0);
            // jnt_ptr_.resize(0);
            // get_jnt_list();
            }
            if(parse_leg_count != 4)
            {
                RCLCPP_ERROR(rclcpp::get_logger("URDF_parser"),"The leg parsed are %d, its not correct",parse_leg_count);
                urdf_usable_=false;
            }
        }
    };


    Axis_Direction Model_Parser::get_ker_dir()
    {
        if(jnts_find_ == 3)
        {
            return Axis_Direction::all;
        }
        else
        {
            Axis_Direction ax = Axis_Direction::not_init;
            std::array<double,3>hfe_dir,kfe_dir;
            std::array<double,3> x_dir = {1.0,0.0,0.0},y_dir={0.0,1.0,0.0},z_dir={0.0,0.0,1.0},x_dir_n = {-1.0,0.0,0.0},y_dir_n={0.0,-1.0,0.0},z_dir_n={0.0,0.0,-1.0};
            for(auto &leg_map:kin_tree_)
            {
                try
                {
                    hfe_dir = leg_map.second[quadruped_jnt_id[1]];
                    kfe_dir = leg_map.second[quadruped_jnt_id[2]];
                }
                catch(std::out_of_range& e)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("URDF_Parser"),"Raised error %s, check the urdf file", e.what());
                    return Axis_Direction::error;
                }
                if(hfe_dir != kfe_dir)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("URDF_Parser"),"The joints direction of leg %s are not correct",leg_map.first.c_str());
                    return Axis_Direction::error;
                }


                if(ax == Axis_Direction::not_init)
                {
                    if(hfe_dir == x_dir || hfe_dir == x_dir_n)
                        ax = Axis_Direction::x;
                    else if(hfe_dir == y_dir || hfe_dir == y_dir_n)
                        ax = Axis_Direction::y;
                    else if(hfe_dir == z_dir || hfe_dir == z_dir_n)
                        ax = Axis_Direction::z;
                    else
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("URDF_Parser"),"The joints direction of leg %s are not correct",leg_map.first.c_str());
                        return Axis_Direction::error;
                    }
                }
                else
                {
                    Axis_Direction act_dir;
                    if(hfe_dir == x_dir || hfe_dir == x_dir_n)
                        act_dir = Axis_Direction::x;
                    else if(hfe_dir == y_dir || hfe_dir == y_dir_n)
                        act_dir = Axis_Direction::y;
                    else if(hfe_dir == z_dir || hfe_dir == y_dir_n )
                        act_dir = Axis_Direction::z;
                    else
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("URDF_Parser"),"The joints direction of leg %s are not correct",leg_map.first.c_str());
                        return Axis_Direction::error;
                    }
                    if(act_dir != ax)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("URDF_Parser"),"The joints direction of leg %s are not correct",leg_map.first.c_str());
                        return Axis_Direction::error;
                    }
                }

            }
            return ax;
        }
        return Axis_Direction::error;
    }

};
