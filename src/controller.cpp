#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <memory>
#include <thread>


#include "assignment1_rt2/action/target.hpp"

namespace target_controller{
class TargetController : public rclcpp::Node{
    public:
    //using for semplicity
    using Target = assignment1_rt2::action::Target;

    explicit TargetController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions() ) : 
    Node("target_controller", options){

        


    }


    private:
    rclcpp_action::Server<Target>::SharedPtr action_server;

    //goal 
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Target::Goal> goal){
            RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
    //cancel
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<Target>> goal_handle){
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }
    //accepted
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Target>> goal_handle){
        std::thread{std::bind(&TargetController::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    //execute function for thread
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Target>> goal_handle){
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Target::Feedback>();
        auto result = std::make_shared<Target::Result>();

        //implementation of action logic
    }



};
}
RCLCPP_COMPONENTS_REGISTER_NODE(target_controller::TargetController)