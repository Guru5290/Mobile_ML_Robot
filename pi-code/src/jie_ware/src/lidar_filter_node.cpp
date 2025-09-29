/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2020-2025, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/* @author Zhang Wanjie                                             */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <limits>
#include <cmath>
#include <string>

class CLidarFilter : public rclcpp::Node
{
public:
    CLidarFilter()
    : Node("lidar_filter_node")
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("source_topic", "/scan");
        this->declare_parameter<std::string>("pub_topic", "/scan_filtered");
        this->declare_parameter<double>("outlier_threshold", 0.1);

        source_topic_name_ = this->get_parameter("source_topic").as_string();
        pub_topic_name_    = this->get_parameter("pub_topic").as_string();
        outlier_threshold_ = this->get_parameter("outlier_threshold").as_double();

        // Publisher
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(pub_topic_name_, 10);

        // Subscriber
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            source_topic_name_, 10,
            std::bind(&CLidarFilter::lidarCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Lidar filter node started. Subscribing to %s, publishing to %s",
                    source_topic_name_.c_str(), pub_topic_name_.c_str());
    }

private:
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        int nRanges = scan->ranges.size();

        // If too few points, just republish
        if (nRanges < 3) 
        {
            scan_pub_->publish(*scan);
            return;
        }

        sensor_msgs::msg::LaserScan new_scan = *scan; // copy entire message

        // Outlier removal
        for (int i = 1; i < nRanges - 1; ++i)
        {
            float prev_range = new_scan.ranges[i-1];
            float current_range = new_scan.ranges[i];
            float next_range = new_scan.ranges[i+1];

            bool current_valid = std::isfinite(current_range) &&
                                 current_range >= new_scan.range_min &&
                                 current_range <= new_scan.range_max;

            if (!current_valid) 
            {
                continue;
            }

            if (std::abs(current_range - prev_range) > outlier_threshold_ &&
                std::abs(current_range - next_range) > outlier_threshold_)
            {
                new_scan.ranges[i] = std::numeric_limits<float>::infinity();
                if (!new_scan.intensities.empty() && i < new_scan.intensities.size()) 
                {
                    new_scan.intensities[i] = 0.0f;
                }
            }
        }

        scan_pub_->publish(new_scan);
    }

    // Members
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    std::string source_topic_name_;
    std::string pub_topic_name_;
    double outlier_threshold_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CLidarFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
