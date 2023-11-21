#ifndef NAV_CONTROL__GETLINE_HPP_
#define NAV_CONTROL__GETLINE_HPP_


#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav_control{

class GetLine : public BT::ActionNodeBase{
    public:
        explicit GetLine(
            const std::string & xml_tag_name,
            const BT::NodeConfiguration & conf
        );

        void halt();
        BT::NodeStatus tick();

        static BT::PortsList providedPorts(){
            return BT::PortsList({});
        };
        void status_callback(const std_msgs::msg::Bool::SharedPtr status);

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr status_sub_;
        std_msgs::msg::Bool last_status_msg;
        rclcpp::Time start_time_;
};
}

#endif