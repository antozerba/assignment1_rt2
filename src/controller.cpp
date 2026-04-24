#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <memory>
#include <thread>
#include <climits>


#include "custom_interface/action/target.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.h"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/utils.h"
#include "math.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

//using for semplicity
using Target = custom_interface::action::Target;


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

        //Odom sub
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&TargetController::odom_callback, this, std::placeholders::_1));

        //Odom tf broadcaster 
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);









    }


    private:
    rclcpp_action::Server<Target>::SharedPtr action_server;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::TransformStamped odom_t;


    //odom callback to publish tf odom-robot
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){

        odom_t.header.stamp = this->get_clock()->now();
        odom_t.header.frame_id = "odom";
        odom_t.child_frame_id = "base_footprint";

        odom_t.transform.translation.x = msg->pose.pose.position.x;
        odom_t.transform.translation.y = msg->pose.pose.position.y;
        odom_t.transform.translation.z = msg->pose.pose.position.z;

        odom_t.transform.rotation = msg->pose.pose.orientation;

        tf_broadcaster_->sendTransform(odom_t);
    }


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

        //CONTROLLER  DOUBLE PHASE
        float distance = LONG_MAX;
        float yaw_error = 360.0;
        rclcpp::Rate rate(10);

        // ---- FASE 1: raggiungi la posizione (x, y) ----
        while (rclcpp::ok() && distance > 0.1) {
            try {
                t = tf_buffer->lookupTransform("base_link", "target", tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
                goal_handle->abort(result);
                return;
            }

            distance = sqrt(pow(t.transform.translation.x, 2) + pow(t.transform.translation.y, 2));
            float heading_error = atan2(t.transform.translation.y, t.transform.translation.x);

            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x  = 0.5 * distance;
            cmd_vel.angular.z = 1.0 * heading_error;
            vel_pub->publish(cmd_vel);
            // feedback
            feedback->partial_pose = {
                static_cast<float>(odom_t.transform.translation.x),
                static_cast<float>(odom_t.transform.translation.y),
                static_cast<float>(tf2::getYaw(odom_t.transform.rotation) * 180.0 / M_PI)
            };
            goal_handle->publish_feedback(feedback);
            rate.sleep();
        }

        // stop
        vel_pub->publish(geometry_msgs::msg::Twist{});

        // ---- FASE 2: ruota verso theta finale ----
        // Il target TF ha già la rotazione finale (M_PI*theta/180)
        // quindi basta leggere lo yaw residuo dal transform base_link->target
        while (rclcpp::ok() && fabs(yaw_error) > 0.01) {  
            try {
                t = tf_buffer->lookupTransform("base_link", "target", tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
                goal_handle->abort(result);
                return;
            }
            yaw_error = tf2::getYaw(t.transform.rotation);  // rotazione residua

            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x  = 0.0;
            cmd_vel.angular.z = 1.0 * yaw_error;
            vel_pub->publish(cmd_vel);
            rate.sleep();
            feedback->partial_pose = {
                static_cast<float>(odom_t.transform.translation.x),
                static_cast<float>(odom_t.transform.translation.y),
                static_cast<float>(tf2::getYaw(odom_t.transform.rotation) * 180.0 / M_PI)
            };
            goal_handle->publish_feedback(feedback);
        }

        vel_pub->publish(geometry_msgs::msg::Twist{});
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");


        
    }

};
}
// int main(int argc, char* argv[]){
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<target_controller::TargetController>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
RCLCPP_COMPONENTS_REGISTER_NODE(target_controller::TargetController)