#include "brewer/tree_nodes.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

class BrewerNode : public rclcpp::Node {
public:
    BrewerNode() : rclcpp::Node("coffee_brewer") {
        // Register nodes
        factory_.registerNodeType<FillUpCoffeeMug>("FillUpCoffeeMug");
        factory_.registerNodeType<BrewEspresso>("BrewEspresso");
        factory_.registerNodeType<BrewCappucino>("BrewCappucino");
        factory_.registerNodeType<BrewLatte>("BrewLatte");
        factory_.registerNodeType<CleanMachine>("CleanMachine");
        factory_.registerNodeType<UserWantsCoffee>("UserWantsCoffee");
        factory_.registerNodeType<IsCleaningMode>("IsCleaningMode");

        brew_subscriber_ = this->create_subscription<std_msgs::msg::String>
            ("/brew", 10, std::bind(&BrewerNode::brew_coffee, this, std::placeholders::_1));
        clean_subscriber_ = this->create_subscription<std_msgs::msg::Bool>
            ("/clean", 10, std::bind(&BrewerNode::handle_clean, this, std::placeholders::_1));
    }
private:
    void brew_coffee(const std_msgs::msg::String & coffee) {
        auto blackboard = BT::Blackboard::create({});
        blackboard->set("coffee_type", coffee.data);
        blackboard->set("user_wants_coffee", true);
        blackboard->set("clean_after_brewing", clean_after_brewing_);
        auto tree = factory_.createTreeFromFile("./src/brewer/include/brewer/brewer_behavior_tree.xml", blackboard);
        BT::NodeStatus status = tree.tickRoot();
    }
    void handle_clean(const std_msgs::msg::Bool & clean) {
        clean_after_brewing_ = clean.data;
    }

    BT::BehaviorTreeFactory factory_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr brew_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr clean_subscriber_;
    bool clean_after_brewing_ = false;
};

int main(int argc, char * argv[]) {
    // Simulate setting global variables and executing the tree
    user_wants_coffee = true;
    cleaning_mode = false;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BrewerNode>());
    rclcpp::shutdown();

    return 0;
}