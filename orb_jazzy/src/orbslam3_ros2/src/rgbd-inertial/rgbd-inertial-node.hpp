#ifndef __RGBD_INERTIAL_NODE_HPP__
#define __RGBD_INERTIAL_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <cv_bridge/cv_bridge.hpp>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

using ImuMsg = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::Image;

class RgbdInertialNode : public rclcpp::Node
{
public:
    RgbdInertialNode(ORB_SLAM3::System* pSLAM);
    ~RgbdInertialNode();

private:
    typedef message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg> approximate_sync_policy;

    void GrabImu(const ImuMsg::SharedPtr msg);
    void GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD);
    void SyncWithImu();
    void PublishPose(const Sophus::SE3f &pose, const rclcpp::Time &timestamp);
    void PublishMapPoints(const std::vector<ORB_SLAM3::MapPoint*> &map_points);

    ORB_SLAM3::System *SLAM_;
    std::thread *syncThread_;

    // IMU subscription
    rclcpp::Subscription<ImuMsg>::SharedPtr subImu_;

    // RGB-D subscriptions with message filters
    std::shared_ptr<message_filters::Subscriber<ImageMsg>> rgb_sub_;
    std::shared_ptr<message_filters::Subscriber<ImageMsg>> depth_sub_;
    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate_;

    // Publishers for visualization
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mapPointsPub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;

    // IMU buffer
    queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex bufMutex_;

    // RGB-D buffer
    struct RgbdData {
        cv::Mat rgb;
        cv::Mat depth;
        double timestamp;
    };
    queue<RgbdData> rgbdBuf_;
    std::mutex bufMutexRgbd_;

    // Path for visualization
    nav_msgs::msg::Path path_;
};

#endif
