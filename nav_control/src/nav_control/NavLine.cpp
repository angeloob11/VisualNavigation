#include <iostream>
#include <string>
#include <set>
#include <algorithm>
#include <fstream>
#include "nav_control/NavLine.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"


namespace nav_control{

    using namespace std::placeholders;
    int NavLine::n_line_ = 0;

    NavLine::NavLine(
        const std::string & xml_tag_name,
        const BT::NodeConfiguration & conf
    )
    : BT::ActionNodeBase(xml_tag_name, conf){
        config().blackboard->get("node",node_);
        vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/output_vel", 100);
        theta_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
            "/theta", 100, std::bind(&NavLine::theta_callback, this, _1));
        status_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/finish", 100, std::bind(&NavLine::status_callback, this, _1));
        
    }

    void
    NavLine::theta_callback(const std_msgs::msg::Float64::SharedPtr theta){
        last_theta_reading = *theta;
    }

    void 
    NavLine::status_callback(const std_msgs::msg::Bool::SharedPtr status_node){
        last_status_reading = *status_node;
    }

    void NavLine::halt(){
    }

    BT::NodeStatus NavLine::tick(){

        std::cout<<"Nav Line Node Active" <<std::endl;

        geometry_msgs::msg::Twist vel_msg;
        
        if (!last_status_reading.data)
        {
            vel_msg.angular.z = -1 * last_theta_reading.data * OMEGA_MAX / THETA_MAX;
            vel_msg.linear.x = 0.5;
            vel_pub_->publish(vel_msg);
            std::string File_Name = "theta_data_";
            File_Name += std::to_string(n_line_);
            File_Name += ".csv";
            CSVfile.open(File_Name, std::ios::out | std::ios::app);
            CSVfile << last_theta_reading.data <<"\n";
            CSVfile.close();

            return BT::NodeStatus::RUNNING;
        }
        else{
            n_line_++;
            return BT::NodeStatus::SUCCESS;
        }        
    }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory){
    factory.registerNodeType<nav_control::NavLine>("NavLine");
}