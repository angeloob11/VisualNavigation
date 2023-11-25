#include <iostream>
#include <string>
#include <set>
#include <algorithm>

#include "nav_control/NavLine.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace nav_control{

    using namespace std::placeholders;

    NavLine::NavLine(
        const std::string & xml_tag_name,
        const BT::NodeConfiguration & conf
    )
    : BT::ActionNodeBase(xml_tag_name, conf){
        config().blackboard->get("node",node_);
        vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/output_vel", 100);
        img_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
            "/camera_img", 100, std::bind(&NavLine::img_callback, this, _1)
            );
        
    }

    void 
    NavLine::img_callback(const sensor_msgs::msg::Image::SharedPtr img){
        last_img_reading_ = *img;
    }
    

    void NavLine::halt(){
    }

    BT::NodeStatus NavLine::tick(){
        std::cout<<"Nav Line Node Active"<< std::endl;
        
        float lenght = 3.0;
        geometry_msgs::msg::Twist vel_msg;
        vel_msg.linear.x = 0.5;
        vel_pub_->publish(vel_msg);

        if (status() == BT::NodeStatus::IDLE)
        {
            start_time_ = node_->now();
        }

        auto elpased = node_->now() - start_time_;
        
        if (elpased < rclcpp::Duration(lenght/0.5, 0))
        {
            return BT::NodeStatus::RUNNING;
        }
        else{
            return BT::NodeStatus::SUCCESS;
        }        
    }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory){
    factory.registerNodeType<nav_control::NavLine>("NavLine");
}