#include <iostream>
#include <string>
#include <set>

#include "nav_control/GetLine.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

namespace nav_control{

    using namespace std::placeholders;
    bool GetLine::status_ = false;

    GetLine::GetLine(
        const std::string & xml_tag_name,
        const BT::NodeConfiguration & conf
    )
    : BT::ActionNodeBase(xml_tag_name, conf){
        config().blackboard->get("node", node_);
        vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/output_vel", 100); 
    }

    void GetLine::halt(){
    }

    BT::NodeStatus GetLine::tick(){
        
        if(status_){
            return BT::NodeStatus::SUCCESS;
        }
        
        std::cout<<"Get Line Node Active"<< std::endl;
        if (status() == BT::NodeStatus::IDLE){
        start_time_ = node_->now();
        }

        geometry_msgs::msg::Twist vel_msg;
        auto elpased = node_->now() - start_time_;

        if (elpased < rclcpp::Duration(10.2415, 0)){
            if (elpased < rclcpp::Duration(5.1, 0)){
                vel_msg.linear.x = 0.5;
                vel_pub_->publish(vel_msg);
                return BT::NodeStatus::RUNNING;
            }
            else if (elpased < rclcpp::Duration(8.2415, 0)){
                vel_msg.angular.z = 0.5;
                vel_pub_->publish(vel_msg);
                return BT::NodeStatus::RUNNING;
            }
            else{
                vel_msg.linear.x = 0.5;
                vel_pub_->publish(vel_msg);
                return BT::NodeStatus::RUNNING;
            }   
        }
        else{
            status_ = true;
            return BT::NodeStatus::SUCCESS;
       }
    }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory){
    factory.registerNodeType<nav_control::GetLine>("GetLine");
}