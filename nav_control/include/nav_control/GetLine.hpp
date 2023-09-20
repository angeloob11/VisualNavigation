#ifndef NAV_CONTROL__GETLINE_HPP_
#define NAV_CONTROL__GETLINE_HPP_


#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"


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

    private:
        int counter_;
};
}

#endif