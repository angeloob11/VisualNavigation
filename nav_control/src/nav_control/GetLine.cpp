#include <iostream>
#include <string>
#include <set>

#include "nav_control/GetLine.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

namespace nav_control{
    GetLine::GetLine(
        const std::string & xml_tag_name,
        const BT::NodeConfiguration & conf
    )
    : BT::ActionNodeBase(xml_tag_name, conf), counter_(0){
    }

    void GetLine::halt(){
    }

    BT::NodeStatus GetLine::tick(){
        //std::cout<<"Get Line Node Active"<< counter_ << std::endl;
        if (counter_++ < 50)
        {
            return BT::NodeStatus::RUNNING;
        }
        else{
            counter_ = 0;
            return BT::NodeStatus::SUCCESS;
        }        
    }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory){
    factory.registerNodeType<nav_control::GetLine>("GetLine");
}