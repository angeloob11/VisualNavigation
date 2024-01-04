#ifndef NAV_CONTROL__CHANGELINE_HPP_
#define NAV_CONTROL__CHANGELINE_HPP_


#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"


namespace nav_control{

class ChangeLine : public BT::ActionNodeBase{
    public:
        explicit ChangeLine(
            const std::string & xml_tag_name,
            const BT::NodeConfiguration & conf
        );

        void halt();
        BT::NodeStatus tick();

        static BT::PortsList providedPorts(){
            return BT::PortsList({});
        };

    private:
        
        rclcpp::Node::SharedPtr node_;
        rclcpp::Time start_time_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
        static int direction_;
};
}

#endif