#include "sirius_base/sirius_base.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace sirius_base
{
    SiriusBase::SiriusBase(const rclcpp::NodeOptions & options) : Node("sirius_base_node", options)
    {
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>("wheel_odom", rclcpp::QoS(10).best_effort(), std::bind(&SiriusBase::pose_callback, this, std::placeholders::_1));
        
        odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&SiriusBase::process, this));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    }

    void SiriusBase::pose_callback(geometry_msgs::msg::Pose::SharedPtr data){
        pose_ = *data;
        // RCLCPP_INFO(this->get_logger(),"pose_data_X : %f", data->position.x);
    }

    void SiriusBase::process(void){
        auto odometry = nav_msgs::msg::Odometry();
        odometry.header.stamp = this->get_clock()->now();
        odometry.header.frame_id = "odom";
        odometry.child_frame_id = "base_footprint";
        odometry.pose.pose = pose_;

        // base_footprint_transformed_.header.stamp = odometry.header.stamp;
        base_footprint_transformed_.header.stamp = this->get_clock()->now();
        base_footprint_transformed_.header.frame_id = "odom";
        base_footprint_transformed_.child_frame_id = "base_footprint";
        base_footprint_transformed_.transform.translation.x = pose_.position.x;
        base_footprint_transformed_.transform.translation.y = pose_.position.y;
        base_footprint_transformed_.transform.translation.z = pose_.position.z;
        base_footprint_transformed_.transform.rotation.x = pose_.orientation.x;
        base_footprint_transformed_.transform.rotation.y = pose_.orientation.y;
        base_footprint_transformed_.transform.rotation.z = pose_.orientation.z;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "base_footprint";
        t.child_frame_id = "base_link";

        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.3;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;

        tf_static_broadcaster_->sendTransform(t);
        tf_broadcaster_->sendTransform(base_footprint_transformed_);
        odometry_publisher_->publish(odometry);
    }

    SiriusBase::~SiriusBase(void){}
}

RCLCPP_COMPONENTS_REGISTER_NODE(sirius_base::SiriusBase)