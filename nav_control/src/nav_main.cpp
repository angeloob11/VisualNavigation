#include <string>
#include <memory>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int args, char * argv[]){
    rclcpp::init(args, argv);

    auto node = rclcpp::Node::make_shared("nav_control_node");

    BT::BehaviorTreeFactory factory;
    BT::SharedLibrary loader;

    factory.registerFromPlugin(loader.getOSName("nav_control_battery_checker_bt_node"));
    factory.registerFromPlugin(loader.getOSName("nav_control_recharge_bt_node"));
    factory.registerFromPlugin(loader.getOSName("nav_control_get_line_bt_node"));
    factory.registerFromPlugin(loader.getOSName("nav_control_nav_line_bt_node"));
    factory.registerFromPlugin(loader.getOSName("nav_control_change_line_bt_node"));

    std::string pkpath = ament_index_cpp::get_package_share_directory("nav_control");
    std::string xml_file = pkpath + "/behavior_tree_xml/nav_control_bt.xml";

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);
    BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

    auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 2666, 2667);

    rclcpp::Rate rate(10);
    bool finish = false;

    while (!finish && rclcpp::ok())
    {
        finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;    
}