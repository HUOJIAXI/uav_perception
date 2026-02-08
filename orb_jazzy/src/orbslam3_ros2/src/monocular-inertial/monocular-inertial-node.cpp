#include "monocular-inertial-node.hpp"

#include <opencv2/core/core.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using std::placeholders::_1;

MonocularInertialNode::MonocularInertialNode(ORB_SLAM3::System *pSLAM) :
    Node("ORB_SLAM3_Monocular_Inertial"),
    SLAM_(pSLAM)
{
    // Declare parameters with default values for Isaac Sim
    this->declare_parameter<std::string>("imu_topic", "/telemetry/imu");
    this->declare_parameter<std::string>("camera_topic", "/pegasus_1/rgb");

    // Get parameter values
    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    std::string camera_topic = this->get_parameter("camera_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "Subscribing to IMU: %s", imu_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribing to Camera: %s", camera_topic.c_str());

    // Create subscriptions with SensorDataQoS (critical for compatibility!)
    subImu_ = this->create_subscription<ImuMsg>(
        imu_topic,
        rclcpp::SensorDataQoS(),
        std::bind(&MonocularInertialNode::GrabImu, this, _1));

    subImg_ = this->create_subscription<ImageMsg>(
        camera_topic,
        rclcpp::SensorDataQoS(),
        std::bind(&MonocularInertialNode::GrabImage, this, _1));

    // Create publishers for visualization
    posePub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("orbslam3/camera_pose", 10);
    pathPub_ = this->create_publisher<nav_msgs::msg::Path>("orbslam3/trajectory", 10);
    mapPointsPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("orbslam3/map_points", 10);
    tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Initialize path
    path_.header.frame_id = "world";

    syncThread_ = new std::thread(&MonocularInertialNode::SyncWithImu, this);

    RCLCPP_INFO(this->get_logger(), "Monocular-Inertial SLAM node initialized with ROS2 visualization");
}

MonocularInertialNode::~MonocularInertialNode()
{
    // Delete sync thread
    syncThread_->join();
    delete syncThread_;

    // Stop all threads
    SLAM_->Shutdown();

    // Save camera trajectory
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    RCLCPP_INFO(this->get_logger(), "Trajectory saved to KeyFrameTrajectory.txt");
}

void MonocularInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    bufMutex_.lock();
    imuBuf_.push(msg);
    static int imu_count = 0;
    if (++imu_count % 100 == 0)
    {
        RCLCPP_INFO(this->get_logger(), "Received %d IMU messages, buffer size: %zu", imu_count, imuBuf_.size());
    }
    bufMutex_.unlock();
}

void MonocularInertialNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    bufMutexImg_.lock();

    if (!imgBuf_.empty())
        imgBuf_.pop();
    imgBuf_.push(msg);

    static int img_count = 0;
    img_count++;
    RCLCPP_INFO(this->get_logger(), "Received image #%d at %.3f, buffer size: %zu",
                img_count, Utility::StampToSec(msg->header.stamp), imgBuf_.size());

    bufMutexImg_.unlock();
}

cv::Mat MonocularInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    return cv_ptr->image.clone();
}

void MonocularInertialNode::SyncWithImu()
{
    static double last_image_time = 0;
    const double imu_wait_time = 0.05; // 50ms tolerance

    RCLCPP_INFO(this->get_logger(), "Sync thread started, waiting for images and IMU data...");

    while (rclcpp::ok())
    {
        cv::Mat im;
        double tIm = 0;

        if (!imgBuf_.empty() && !imuBuf_.empty())
        {
            tIm = Utility::StampToSec(imgBuf_.front()->header.stamp);

            // Wait for sufficient IMU data (at least 2 measurements after the image)
            bufMutex_.lock();
            double imu_back_time = Utility::StampToSec(imuBuf_.back()->header.stamp);
            int imu_count = imuBuf_.size();
            bufMutex_.unlock();

            if (tIm > imu_back_time - imu_wait_time)
            {
                // Not enough IMU data yet, wait a bit
                std::chrono::milliseconds tSleep(2);
                std::this_thread::sleep_for(tSleep);
                continue;
            }

            bufMutexImg_.lock();
            im = GetImage(imgBuf_.front());
            imgBuf_.pop();
            bufMutexImg_.unlock();

            // Collect IMU measurements between previous and current frame
            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufMutex_.lock();
            if (!imuBuf_.empty())
            {
                vImuMeas.clear();
                // Collect all IMU data between last_image_time and current image time
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tIm)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);

                    // Only include IMU data newer than last image (except for first frame)
                    if (last_image_time == 0 || t > last_image_time)
                    {
                        cv::Point3f acc(imuBuf_.front()->linear_acceleration.x,
                                       imuBuf_.front()->linear_acceleration.y,
                                       imuBuf_.front()->linear_acceleration.z);
                        cv::Point3f gyr(imuBuf_.front()->angular_velocity.x,
                                       imuBuf_.front()->angular_velocity.y,
                                       imuBuf_.front()->angular_velocity.z);
                        vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    }
                    imuBuf_.pop();
                }
            }
            bufMutex_.unlock();

            // Update last image time
            last_image_time = tIm;

            // Log IMU measurements for debugging
            if (vImuMeas.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Empty IMU measurements! Image: %.3f, IMU count: %d",
                           tIm, imu_count);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Tracking frame at %.3f with %zu IMU measurements",
                           tIm, vImuMeas.size());
            }

            // Track with IMU measurements
            RCLCPP_INFO(this->get_logger(), "Calling TrackMonocular...");
            Sophus::SE3f Tcw = SLAM_->TrackMonocular(im, tIm, vImuMeas);

            int tracking_state = SLAM_->GetTrackingState();
            RCLCPP_INFO(this->get_logger(), "Tracking state: %d", tracking_state);

            // Publish pose and map if tracking succeeded
            if (!Tcw.translation().isZero())
            {
                rclcpp::Time ros_time(static_cast<int64_t>(tIm * 1e9));
                PublishPose(Tcw, ros_time);

                // Publish map points periodically (every 10 frames to reduce overhead)
                static int frame_count = 0;
                if (++frame_count % 10 == 0)
                {
                    auto tracked_points = SLAM_->GetTrackedMapPoints();
                    PublishMapPoints(tracked_points);
                }
            }

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
        else
        {
            static int wait_count = 0;
            if (++wait_count % 1000 == 0)
            {
                bufMutexImg_.lock();
                bufMutex_.lock();
                RCLCPP_INFO(this->get_logger(), "Waiting... Image buffer: %zu, IMU buffer: %zu",
                           imgBuf_.size(), imuBuf_.size());
                bufMutex_.unlock();
                bufMutexImg_.unlock();
            }
            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Sync thread exiting");
}

void MonocularInertialNode::PublishPose(const Sophus::SE3f &Tcw, const rclcpp::Time &timestamp)
{
    // Convert camera-to-world to world-to-camera
    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Vector3f position = Twc.translation();
    Eigen::Quaternionf orientation = Twc.unit_quaternion();

    // Publish PoseStamped
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = timestamp;
    pose_msg.header.frame_id = "world";
    pose_msg.pose.position.x = position.x();
    pose_msg.pose.position.y = position.y();
    pose_msg.pose.position.z = position.z();
    pose_msg.pose.orientation.x = orientation.x();
    pose_msg.pose.orientation.y = orientation.y();
    pose_msg.pose.orientation.z = orientation.z();
    pose_msg.pose.orientation.w = orientation.w();
    posePub_->publish(pose_msg);

    // Add to path and publish
    path_.header.stamp = timestamp;
    path_.poses.push_back(pose_msg);
    // Limit path length to prevent memory issues
    if (path_.poses.size() > 1000)
    {
        path_.poses.erase(path_.poses.begin());
    }
    pathPub_->publish(path_);

    // Publish TF
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = timestamp;
    transform.header.frame_id = "world";
    transform.child_frame_id = "camera";
    transform.transform.translation.x = position.x();
    transform.transform.translation.y = position.y();
    transform.transform.translation.z = position.z();
    transform.transform.rotation.x = orientation.x();
    transform.transform.rotation.y = orientation.y();
    transform.transform.rotation.z = orientation.z();
    transform.transform.rotation.w = orientation.w();
    tfBroadcaster_->sendTransform(transform);
}

void MonocularInertialNode::PublishMapPoints(const std::vector<ORB_SLAM3::MapPoint*> &map_points)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = "world";

    // Set up point cloud structure
    cloud_msg.height = 1;
    cloud_msg.width = 0;
    cloud_msg.is_dense = false;
    cloud_msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(map_points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    for (const auto &mp : map_points)
    {
        if (mp && !mp->isBad())
        {
            Eigen::Vector3f pos = mp->GetWorldPos();
            *iter_x = pos.x();
            *iter_y = pos.y();
            *iter_z = pos.z();
            ++iter_x;
            ++iter_y;
            ++iter_z;
            cloud_msg.width++;
        }
    }

    mapPointsPub_->publish(cloud_msg);
}
