#include <iostream>
#include <string>
#include <set>

#include "nav_control/Recharge.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace nav_control{
    Recharge::Recharge(
        const std::string & xml_tag_name,
        const BT::NodeConfiguration & conf
    )
    : BT::ActionNodeBase(xml_tag_name, conf), counter_(0){
        config().blackboard->get("node", node_);
        vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/output_vel", 100);
    }

    void Recharge::halt(){
    }

    BT::NodeStatus Recharge::tick(){
        std::cout<<"Recharge node active"<< counter_ << std::endl;
        geometry_msgs::msg::Twist vel_msg_;
        vel_msg_.linear.x = 0;
        vel_msg_.angular.z = 0;
        vel_pub_->publish(vel_msg_);
        if (counter_++ < 50)
        {
            return BT::NodeStatus::RUNNING;
        }
        else{
            counter_ = 0;
            config().blackboard->set<float>("battery_level", 100.0f);
            return BT::NodeStatus::SUCCESS;
        }        
    }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory){
    factory.registerNodeType<nav_control::Recharge>("Recharge");
}