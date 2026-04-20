#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <memory>
#include <thread>
#include <climits>


#include "assignment1_rt2/action/target.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.h"

//using for semplicity
using Target = assignment1_rt2::action::Target;


namespace target_controller{
class TargetController : public rclcpp::Node{
    public:

    explicit TargetController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions() ) : 
    Node("target_controller", options){

        //Action 

        this-> action_server = rclcpp_action::create_server<Target>(
            this,
            "target",
            std::bind(&TargetController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&TargetController::handle_cancel, this, std::placeholders::_1),
            std::bind(&TargetController::handle_accepted, this, std::placeholders::_1)
        );

        //Frame Listerner
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener  = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        //VelPub
        vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);







    }


    private:
    rclcpp_action::Server<Target>::SharedPtr action_server;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;


    //goal 
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Target::Goal> goal){
            RCLCPP_INFO(this->get_logger(), "Received goal: X: %f, Y: %f, Theta: %f",
             goal->target_pose[0], goal->target_pose[1], goal->target_pose[2]);
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

        geometry_msgs::msg::TransformStamped t;

        //implementation of action logic:
        float distance = LONG_MAX;
        float angle = -1.0;
        while(rclcpp::ok() && distance > 0.1){
            //getting trasf of robot-target
            try{
                t  = tf_buffer->lookupTransform("base_link", "target", tf2::TimePointZero);
            }catch (const tf2::TransformException & ex){
                RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
                goal_handle->abort(result);
                return;
            }


            //compute controller based on the distance
            distance = static_cast<float>(sqrt(pow(t.transform.translation.x, 2) + pow(t.transform.translation.y, 2)));
            angle = atan2(t.transform.translation.y, t.transform.translation.x); 

            //send feedback
            feedback->partial_pose = {static_cast<float>(t.transform.translation.x), static_cast<float>(t.transform.translation.y), angle};
            goal_handle->publish_feedback(feedback);
        
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.5 * distance;
            cmd_vel.angular.z = 1.0 * angle;

            vel_pub->publish(cmd_vel);

        }
        if(rclcpp::ok())
        {
            //stopping the robot
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            vel_pub->publish(cmd_vel);

            //setting result
            result->final_pose = {static_cast<float>(t.transform.translation.x), static_cast<float>(t.transform.translation.y), angle};
            goal_handle->succeed(result);
        }








    }



};
}
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<target_controller::TargetController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
RCLCPP_COMPONENTS_REGISTER_NODE(target_controller::TargetController)