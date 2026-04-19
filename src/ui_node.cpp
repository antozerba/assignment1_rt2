#include <memory>
#include "stdio.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"


class TargetInferface  : public rclcpp::Node{
    public: 
    TargetInferface() : Node("target_interface"){
        target_broadcaster =  std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        //publish targret pos
        this->get_input();
        this->make_target();

        //sleep for rviz 
        rclcpp::sleep_for(std::chrono::seconds(1));
    };

    private: 
    void get_input(){
        std::cout << "Enter target position (x y theta): ";
        std::cin >> x >> y >> theta;
    }

    void make_target(){
        geometry_msgs::msg::TransformStamped target_transform;
        target_transform.header.stamp = this->get_clock()->now();
        target_transform.header.frame_id = "base_link"; //father link
        target_transform.child_frame_id = "target";

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




};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetInferface>());
    rclcpp::shutdown();
    return 0;
}