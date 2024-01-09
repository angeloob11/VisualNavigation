#ifndef NAV_CONTROL__NAVLINE_HPP_
#define NAV_CONTROL__NAVLINE_HPP_


#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

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

        void theta_callback(const std_msgs::msg::Float64::SharedPtr theta);
        void status_callback(const std_msgs::msg::Bool::SharedPtr status_node);
        const float THETA_MAX = 1.57079;
        const float OMEGA_MAX = 0.5;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr theta_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr status_sub_;
        std_msgs::msg::Float64 last_theta_reading;
        std_msgs::msg::Bool last_status_reading;
};
}

#endif