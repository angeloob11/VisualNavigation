#ifndef NAV_CONTROL__NAVLINE_HPP_
#define NAV_CONTROL__NAVLINE_HPP_


#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace nav_control{

class NavLine : public BT::ActionNodeBase{
    public:
        explicit NavLine(
            const std::string & xml_tag_name,
            const BT::NodeConfiguration & conf
        );

        void halt();
        BT::NodeStatus tick();

        static BT::PortsList providedPorts(){
            return BT::PortsList({});
        };

        void img_callback(const sensor_msgs::msg::Image::SharedPtr img);

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Time start_time_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
        sensor_msgs::msg::Image last_img_reading_;
};
}

#endif