#ifndef NAV_CONTROL__RECHARGE_HPP_
#define NAV_CONTROL__RECHARGE_HPP_


#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


namespace nav_control{

class Recharge : public BT::ActionNodeBase{
    public:
        explicit Recharge(
            const std::string & xml_tag_name,
            const BT::NodeConfiguration & conf
        );

        void halt();
        BT::NodeStatus tick();

        static BT::PortsList providedPorts(){
            return BT::PortsList({});
        };

    private:
        int counter_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

};
}

#endif