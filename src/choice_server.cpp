#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

class ChoiceServer : public rclcpp::Node
{
public:
    ChoiceServer() : Node("choice_server"), is_fish_(true)
    {
        set_is_fish_srv_ = this->create_service<std_srvs::srv::SetBool>(
            "choice/set_is_fish",
            std::bind(
                &ChoiceServer::handle_set_is_fish,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
        
        get_is_fish_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "choice/get_is_fish",
            std::bind(
                &ChoiceServer::handle_get_is_fish,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "[choice] ready, init to fish");
    }

private:
    void handle_set_is_fish(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        is_fish_ = request->data;
        response->success = true;
        response->message = std::string(is_fish_ ? "fish" : "shark");
        RCLCPP_INFO(
            this->get_logger(),
            "[choice] set to %s",
            is_fish_ ? "fish" : "shark");
    }

    void handle_get_is_fish(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request; // suppress unused variable warning
        response->success = is_fish_;
        response->message = std::string(is_fish_ ? "fish" : "shark");
        RCLCPP_INFO(
            this->get_logger(),
            "[choice] responding to query with %s",
            is_fish_ ? "fish" : "shark");
    }

    bool is_fish_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_is_fish_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_is_fish_srv_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChoiceServer>());
    rclcpp::shutdown();
    return 0;
}
