#ifndef __MONOCULAR_INERTIAL_NODE_HPP__
#define __MONOCULAR_INERTIAL_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <cv_bridge/cv_bridge.hpp>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

using ImuMsg = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::Image;

class MonocularInertialNode : public rclcpp::Node
{
public:
    MonocularInertialNode(ORB_SLAM3::System* pSLAM);
    ~MonocularInertialNode();

private:
    void GrabImu(const ImuMsg::SharedPtr msg);
    void GrabImage(const ImageMsg::SharedPtr msg);
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);
    void SyncWithImu();
    void PublishPose(const Sophus::SE3f &pose, const rclcpp::Time &timestamp);
    void PublishMapPoints(const std::vector<ORB_SLAM3::MapPoint*> &map_points);

    rclcpp::Subscription<ImuMsg>::SharedPtr   subImu_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImg_;

    // Publishers for visualization
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mapPointsPub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;

    ORB_SLAM3::System *SLAM_;
    std::thread *syncThread_;

    // IMU
    queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex bufMutex_;

    // Image
    queue<ImageMsg::SharedPtr> imgBuf_;
    std::mutex bufMutexImg_;

    // Path for visualization
    nav_msgs::msg::Path path_;
};

#endif
