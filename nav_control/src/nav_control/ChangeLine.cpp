#include <iostream>
#include <string>
#include <set>

#include "nav_control/ChangeLine.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/twist.h"
#include "rclcpp/rclcpp.hpp"

namespace nav_control{

    int ChangeLine::direction_ = -1;

    ChangeLine::ChangeLine(
        const std::string & xml_tag_name,
        const BT::NodeConfiguration & conf
    )
    : BT::ActionNodeBase(xml_tag_name, conf){
        config().blackboard->get("node", node_);
        vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/output_vel", 100); 
    }

    void ChangeLine::halt(){
    }

    BT::NodeStatus ChangeLine::tick(){
        std::cout<<"Change Line Node Active"<< std::endl;
        if (status() == BT::NodeStatus::IDLE)
        {
            start_time_ = node_->now();
        }

        geometry_msgs::msg::Twist vel_msg;
        auto elpased = node_->now() - start_time_;

        if (elpased < rclcpp::Duration(13.8831, 0)){
            if(elpased < rclcpp::Duration(1, 0)){
                vel_msg.linear.x = 0.5;
                vel_pub_->publish(vel_msg);
                return BT::NodeStatus::RUNNING;
            }
            else if (elpased < rclcpp::Duration(4.1415, 0)){
                vel_msg.angular.z = direction_*0.5;
                vel_pub_->publish(vel_msg);
                return BT::NodeStatus::RUNNING;
            }
            else if (elpased < rclcpp::Duration(9.7415, 0)){
                vel_msg.linear.x = 0.5;
                vel_pub_->publish(vel_msg);
                return BT::NodeStatus::RUNNING;
            }
            else if (elpased < rclcpp::Duration(12.8831, 0)){
                vel_msg.angular.z = direction_*0.5;
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
            direction_ *= -1;
            return BT::NodeStatus::SUCCESS;
        }          
    }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory){
    factory.registerNodeType<nav_control::ChangeLine>("ChangeLine");
}