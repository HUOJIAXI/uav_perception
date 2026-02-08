#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>

#include <rclcpp/rclcpp.hpp>

#include <opencv2/core/core.hpp>

#include "System.h"
#include "rgbd-inertial-node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if(argc < 3)
    {
        std::cerr << std::endl << "Usage: ros2 run orbslam3 rgbd-inertial path_to_vocabulary path_to_settings" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    bool visualization = true;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_RGBD, visualization);

    auto node = std::make_shared<RgbdInertialNode>(&SLAM);

    std::cout << "============================" << std::endl;
    std::cout << "RGB-D-Inertial SLAM node started" << std::endl;
    std::cout << "Topics:" << std::endl;
    std::cout << "  RGB: /pegasus_1/rgb" << std::endl;
    std::cout << "  Depth: /pegasus_1/depth" << std::endl;
    std::cout << "  IMU: /telemetry/imu" << std::endl;
    std::cout << "============================" << std::endl;

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
