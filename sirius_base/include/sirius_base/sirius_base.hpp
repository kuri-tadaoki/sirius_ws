#ifndef SIRIUS_BASE_HPP_
#define SIRIUS_BASE_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define SIRIUS_BASE_EXPORT __attribute__((dllexport))
#define SIRIUS_BASE_IMPORT __attribute__((dllimport))
#else
#define SIRIUS_BASE_EXPORT __declspec(dllexport)
#define SIRIUS_BASE_IMPORT __declspec(dllimport)
#endif
#ifdef SIRIUS_BASE_BUILDING_DLL
#define SIRIUS_BASE_PUBLIC \
  SIRIUS_BASE_EXPORT
#else
#define SIRIUS_BASE_PUBLIC \
  SIRIUS_BASE_IMPORT
#endif
#define SIRIUS_BASE_PUBLIC_TYPE \
  SIRIUS_BASE_PUBLIC
#define SIRIUS_BASE_LOCAL
#else
#define SIRIUS_BASE_EXPORT \
  __attribute__((visibility("default")))
#define SIRIUS_BASE_IMPORT
#if __GNUC__ >= 4
#define SIRIUS_BASE_PUBLIC \
  __attribute__((visibility("default")))
#define SIRIUS_BASE_LOCAL __attribute__((visibility("hidden")))
#else
#define SIRIUS_BASE_PUBLIC
#define SIRIUS_BASE_LOCAL
#endif
#define SIRIUS_BASE_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "
#endif



#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace sirius_base
{
    class SiriusBase : public rclcpp::Node
    {
        public:
            SIRIUS_BASE_PUBLIC
            explicit SiriusBase(const rclcpp::NodeOptions & options);
            virtual ~SiriusBase(void);
        private:
            void pose_callback(geometry_msgs::msg::Pose::SharedPtr data);
            void process(void);

            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
            rclcpp::TimerBase::SharedPtr timer_;

            rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;
            geometry_msgs::msg::Pose pose_;

            std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
            std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

            geometry_msgs::msg::TransformStamped base_footprint_transformed_;
            geometry_msgs::msg::TransformStamped t;

    };
}

#endif