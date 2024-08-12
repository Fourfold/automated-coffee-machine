#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <iostream>
#include <filesystem>

// Global user input variables (simulated)
bool user_wants_coffee = false;
bool cleaning_mode = false;

// Action Node to fill up the coffee mug
class FillUpCoffeeMug : public BT::SyncActionNode {
public:
    FillUpCoffeeMug(const std::string& name) : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override {
        // Simulate filling up coffee mug
        std::cout << "Filling up the coffee mug...\nThe coffee is ready!\n\n";
        return BT::NodeStatus::SUCCESS;
    }
};

// Action Node to brew coffee
class BrewCappucino : public BT::SyncActionNode {
public:
    BrewCappucino(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("coffee_type")};
    }

    BT::NodeStatus tick() override {
        BT::Optional<std::string> coffeeType = getInput<std::string>("coffee_type");

        if (!coffeeType) {
            throw BT::RuntimeError("Invalid required input message: ", coffeeType.error());
        }

        std::string type = coffeeType.value();

        if (type == "Cappucino" || type == "cappucino") {
            std::cout << "Brewing cappucino..." << std::endl;
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }
};

// Action Node to brew coffee
class BrewEspresso : public BT::SyncActionNode {
public:
    BrewEspresso(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("coffee_type")};
    }

    BT::NodeStatus tick() override {
        BT::Optional<std::string> coffeeType = getInput<std::string>("coffee_type");

        if (!coffeeType) {
            throw BT::RuntimeError("Invalid required input message: ", coffeeType.error());
        }

        std::string type = coffeeType.value();

        if (type == "Espresso" || type == "espresso") {
            std::cout << "Brewing espresso..." << std::endl;
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }
};

// Action Node to brew coffee
class BrewLatte : public BT::SyncActionNode {
public:
    BrewLatte(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("coffee_type")};
    }

    BT::NodeStatus tick() override {
        BT::Optional<std::string> coffeeType = getInput<std::string>("coffee_type");

        if (!coffeeType) {
            throw BT::RuntimeError("Invalid required input message: ", coffeeType.error());
        }

        std::string type = coffeeType.value();

        if (type == "Latte" || type == "latte") {
            std::cout << "Brewing latte..." << std::endl;
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }
};

// Action Node to clean the machine
class CleanMachine : public BT::SyncActionNode {
public:
    CleanMachine(const std::string& name) : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override {
        // Simulate cleaning the machine
        std::cout << "Cleaning the brewer...\nThe brewer is now clean!\n\n";
        return BT::NodeStatus::SUCCESS;
    }
};

// Condition Node to check if the user wants coffee
class UserWantsCoffee : public BT::ConditionNode {
public:
    UserWantsCoffee(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<bool>("user_wants_coffee")};
    }

    BT::NodeStatus tick() override {
        BT::Optional<bool> coffeeRequested = getInput<bool>("user_wants_coffee");

        if (!coffeeRequested) {
            throw BT::RuntimeError("Invalid required input message: ", coffeeRequested.error());
        }

        bool user_wants_coffee = coffeeRequested.value();

        return user_wants_coffee ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

// Condition Node to check if the machine is in cleaning mode
class IsCleaningMode : public BT::ConditionNode {
public:
    IsCleaningMode(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<bool>("clean_after_brewing")};
    }

    BT::NodeStatus tick() override {
        BT::Optional<bool> cleanRequested = getInput<bool>("clean_after_brewing");

        if (!cleanRequested) {
            throw BT::RuntimeError("Invalid required input message: ", cleanRequested.error());
        }

        bool cleaning_mode = cleanRequested.value();

        return cleaning_mode ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};