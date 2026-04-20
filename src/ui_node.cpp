#include <memory>
#include "stdio.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "assignment1_rt2/action/target.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

//using for semplicityi
using Target = assignment1_rt2::action::Target;
using TargetHandle = rclcpp_action::ClientGoalHandle<Target>;
using namespace std::placeholders;

class TargetInterface  : public rclcpp::Node{
    public: 
    TargetInterface() : Node("target_interface"){
        

        //broadcaster
        target_broadcaster =  std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        //publish targret pos
        this->get_input();
        this->make_target();

        //client
        this->action_client = rclcpp_action::create_client<Target>(this, "target");

        //timer called once, stopped in send_goal()
        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&TargetInterface::send_goal, this));



    };

    private: 
    void send_goal(){
        timer_->cancel(); //stop timer after first call

        if(!this->action_client->wait_for_action_server(std::chrono::seconds(10))){
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown(); //close node if no action server available
            return;
        }
        //creazione goal
        auto goal = Target::Goal();
        goal.target_pose.push_back(x);
        goal.target_pose.push_back(y);
        goal.target_pose.push_back(theta);
        RCLCPP_INFO(this->get_logger(), "Sending goal");
        auto send_goal_options = rclcpp_action::Client<Target>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            std::bind(&TargetInterface::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = 
            std::bind(&TargetInterface::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = 
            std::bind(&TargetInterface::result_callback, this, _1);
        this->action_client->async_send_goal(goal, send_goal_options);

        

    }

    void goal_response_callback(std::shared_ptr<TargetHandle> goal_handle)
    {
        if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(TargetHandle::SharedPtr,
        const std::shared_ptr<const Target::Feedback> feedback)
    {
        //logging partial pose value
        RCLCPP_INFO(this->get_logger(), "Partial Pose X: %f, Y: %f, Theta: %f", 
                feedback->partial_pose[0], feedback->partial_pose[1], feedback->partial_pose[2]);
    }

    void result_callback(const TargetHandle::WrappedResult & result)
    {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
        //logging final pose value
        RCLCPP_INFO(this->get_logger(), "Final Pose X: %f, Y: %f, Theta: %f", 
                result.result->final_pose[0], result.result->final_pose[1], result.result->final_pose[2]);
    }

    void get_input(){
        std::cout << "Enter target position (x y theta): ";
        std::cin >> x >> y >> theta;
    }

    void make_target(){
        geometry_msgs::msg::TransformStamped target_transform;
        target_transform.header.stamp = this->get_clock()->now();
        target_transform.header.frame_id = "base_link"; //father link
        target_transform.child_frame_id = "target"; //child link

        target_transform.transform.translation.x = x;
        target_transform.transform.translation.y = y;
        target_transform.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        target_transform.transform.rotation.x = q.x();
        target_transform.transform.rotation.y = q.y();
        target_transform.transform.rotation.z = q.z();
        target_transform.transform.rotation.w = q.w();

        target_broadcaster->sendTransform(target_transform);
    }

    float x,y,theta;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> target_broadcaster;

    //TODO: client for action
    rclcpp_action::Client<Target>::SharedPtr action_client;
    rclcpp::TimerBase::SharedPtr timer_;





};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetInterface>());
    rclcpp::shutdown();
    return 0;
}