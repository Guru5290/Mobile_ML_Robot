#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <vector>
#include <cmath>
#include <deque>
#include <tuple>

// Use C++11's "using" for cleaner syntax, especially for placeholders
using std::placeholders::_1;

// It's best practice in ROS 2 to encapsulate node logic in a class
class LidarLocalizationNode : public rclcpp::Node
{
public:
    LidarLocalizationNode() : Node("lidar_loc_node")
    {
        // ROS 2 Parameter Declaration with default values
        this->declare_parameter<std::string>("base_frame", "base_footprint");
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("laser_frame", "laser_frame");
        this->declare_parameter<std::string>("laser_topic", "scan");
        this->declare_parameter<int>("update_freq", 30);

        // Get parameters
        this->get_parameter("base_frame", base_frame_);
        this->get_parameter("odom_frame", odom_frame_);
        this->get_parameter("laser_frame", laser_frame_);
        this->get_parameter("laser_topic", laser_topic_);
        this->get_parameter("update_freq", update_freq_);
        
        RCLCPP_INFO(this->get_logger(), "Starting lidar_loc_node with parameters:");
        RCLCPP_INFO(this->get_logger(), "base_frame: %s", base_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "odom_frame: %s", odom_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "laser_frame: %s", laser_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "laser_topic: %s", laser_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "update_freq: %d", update_freq_);

        // ROS 2 QoS (Quality of Service) profiles. Use sensor data for scan, and latching for map.
        auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        auto sensor_qos = rclcpp::SensorDataQoS();

        // ROS 2 Subscribers
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_, map_qos, std::bind(&LidarLocalizationNode::mapCallback, this, _1));
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            laser_topic_, sensor_qos, std::bind(&LidarLocalizationNode::scanCallback, this, _1));

        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10, std::bind(&LidarLocalizationNode::initialPoseCallback, this, _1));

        // ROS 2 Service Client
        clear_costmaps_client_ = this->create_client<std_srvs::srv::Empty>("move_base/clear_costmaps");

        // Initialize TF2 components
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // ROS 2 Timer instead of ros::Rate in a while loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000/update_freq_), 
            std::bind(&LidarLocalizationNode::pose_tf, this));
    }

private:
    // Member variables replace global variables
    nav_msgs::msg::OccupancyGrid map_msg_;
    cv::Mat map_cropped_;
    cv::Mat map_temp_;
    sensor_msgs::msg::RegionOfInterest map_roi_info_;
    std::vector<cv::Point2f> scan_points_;
    
    // ROS 2 handles are smart pointers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_costmaps_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    // TF2
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Parameters
    std::string base_frame_;
    std::string odom_frame_;
    std::string map_frame_ = "map";
    std::string laser_frame_;
    std::string laser_topic_;
    std::string map_topic_ = "map";
    int update_freq_;

    // State variables
    float lidar_x_ = 250, lidar_y_ = 250, lidar_yaw_ = 0;
    const float deg_to_rad = M_PI / 180.0;
    int clear_countdown_ = -1;
    int scan_count_ = 0;
    std::deque<std::tuple<float, float, float>> data_queue_;

    // Callback for initial pose
    // Note the message type is now a shared pointer
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        double map_x = msg->pose.pose.position.x;
        double map_y = msg->pose.pose.position.y;
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        if (map_msg_.info.resolution <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Map info is invalid or not received yet.");
            return;
        }

        lidar_x_ = (map_x - map_msg_.info.origin.position.x) / map_msg_.info.resolution - map_roi_info_.x_offset;
        lidar_y_ = (map_y - map_msg_.info.origin.position.y) / map_msg_.info.resolution - map_roi_info_.y_offset;
        lidar_yaw_ = -yaw;
        clear_countdown_ = 30;
    }

    // void crop_map();
    // void processMap();
    
    // Callback for map
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received map.");
        map_msg_ = *msg;
        crop_map();
        processMap();        
    }
    
    void crop_map()
    {
        auto info = map_msg_.info;
        int xMax,xMin,yMax,yMin;
        xMax=xMin= info.width/2;
        yMax=yMin= info.height/2;
        bool bFirstPoint = true;

        cv::Mat map_raw(info.height, info.width, CV_8UC1, cv::Scalar(128));

        for(unsigned int y = 0; y < info.height; y++) {
            for(unsigned int x = 0; x < info.width; x++) {
                int index = y * info.width + x;
                map_raw.at<uchar>(y, x) = static_cast<uchar>(map_msg_.data[index]);
                if(map_msg_.data[index] == 100) {
                    if(bFirstPoint){
                        xMax = xMin = x;
                        yMax = yMin = y;
                        bFirstPoint = false;
                        continue;
                    }
                    xMin = std::min(xMin, (int)x);
                    xMax = std::max(xMax, (int)x);
                    yMin = std::min(yMin, (int)y);
                    yMax = std::max(yMax, (int)y);
                }
            }
        }
        
        int cen_x = (xMin + xMax)/2;
        int cen_y = (yMin + yMax)/2;
        int new_half_width = abs(xMax - xMin)/2 + 50;
        int new_half_height = abs(yMax - yMin)/2 + 50;
        int new_origin_x = cen_x - new_half_width;
        int new_origin_y = cen_y - new_half_height;
        int new_width = new_half_width*2;
        int new_height = new_half_height*2;

        if(new_origin_x < 0) new_origin_x = 0;
        if((new_origin_x + new_width) > (int)info.width) new_width = info.width - new_origin_x;
        if(new_origin_y < 0) new_origin_y = 0;
        if((new_origin_y + new_height) > (int)info.height) new_height = info.height - new_origin_y;
        
        cv::Rect roi(new_origin_x, new_origin_y, new_width, new_height);
        map_cropped_ = map_raw(roi).clone();

        map_roi_info_.x_offset = new_origin_x;
        map_roi_info_.y_offset = new_origin_y;
        map_roi_info_.width = new_width;
        map_roi_info_.height = new_height;

        // In ROS 2, we pass a shared pointer to the callback
        auto init_pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
        init_pose->pose.pose.position.x = 0.0;
        init_pose->pose.pose.position.y = 0.0;
        init_pose->pose.pose.orientation.w = 1.0;
        initialPoseCallback(init_pose);
    }

    // Callback for laser scan
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        scan_points_.clear();
        double angle = msg->angle_min;

        geometry_msgs::msg::TransformStamped transformStamped;
        try {
            // lookupTransform takes a target frame and a source frame
            // tf2::TimePointZero means get the latest available transform
            transformStamped = tf_buffer_->lookupTransform(base_frame_, laser_frame_, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                laser_frame_.c_str(), base_frame_.c_str(), ex.what());
            return;
        }

        tf2::Quaternion q_lidar;
        tf2::fromMsg(transformStamped.transform.rotation, q_lidar);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q_lidar).getRPY(roll, pitch, yaw);
        const double tolerance = 0.1;
        bool lidar_is_inverted = std::abs(std::abs(roll) - M_PI) < tolerance;
        lidar_is_inverted *= !(std::abs(std::abs(pitch) - M_PI) < tolerance);

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            if (msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max) {
                float x_laser = msg->ranges[i] * cos(angle);
                float y_laser = -msg->ranges[i] * sin(angle);
                
                geometry_msgs::msg::PointStamped point_laser, point_base;
                point_laser.header.frame_id = laser_frame_;
                point_laser.header.stamp = msg->header.stamp;
                point_laser.point.x = x_laser;
                point_laser.point.y = y_laser;

                tf2::doTransform(point_laser, point_base, transformStamped);

                float x = point_base.point.x / map_msg_.info.resolution;
                float y = point_base.point.y / map_msg_.info.resolution;
                if (lidar_is_inverted) {
                    x = -x;
                    y = -y;
                }
                scan_points_.push_back(cv::Point2f(x, y));
            }
            angle += msg->angle_increment;
        }

        if(scan_count_ == 0) scan_count_++;

        // This loop was `while(ros::ok())` which is not a good pattern.
        // The callback should do its work and return. The logic here seems to
        // be an optimization loop, which is fine to keep.
        while (rclcpp::ok()) {
            if (!map_cropped_.empty()) {
                std::vector<cv::Point2f> transform_points, clockwise_points, counter_points;
                int max_sum = 0;
                float best_dx = 0, best_dy = 0, best_dyaw = 0;
                for (const auto& point : scan_points_) {
                    float rotated_x = point.x * cos(lidar_yaw_) - point.y * sin(lidar_yaw_);
                    float rotated_y = point.x * sin(lidar_yaw_) + point.y * cos(lidar_yaw_);
                    transform_points.push_back(cv::Point2f(rotated_x + lidar_x_, lidar_y_ - rotated_y));
                    
                    float clockwise_yaw = lidar_yaw_ + deg_to_rad;
                    rotated_x = point.x * cos(clockwise_yaw) - point.y * sin(clockwise_yaw);
                    rotated_y = point.x * sin(clockwise_yaw) + point.y * cos(clockwise_yaw);
                    clockwise_points.push_back(cv::Point2f(rotated_x + lidar_x_, lidar_y_ - rotated_y));

                    float counter_yaw = lidar_yaw_ - deg_to_rad;
                    rotated_x = point.x * cos(counter_yaw) - point.y * sin(counter_yaw);
                    rotated_y = point.x * sin(counter_yaw) + point.y * cos(counter_yaw);
                    counter_points.push_back(cv::Point2f(rotated_x + lidar_x_, lidar_y_ - rotated_y));
                }

                std::vector<cv::Point2f> offsets = {{0,0}, {1,0}, {-1,0}, {0,1}, {0,-1}};
                std::vector<std::vector<cv::Point2f>> point_sets = {transform_points, clockwise_points, counter_points};
                std::vector<float> yaw_offsets = {0, deg_to_rad, -deg_to_rad};
                
                for (size_t i = 0; i < offsets.size(); ++i) {
                    for (size_t j = 0; j < point_sets.size(); ++j) {
                        int sum = 0;
                        for (const auto& point : point_sets[j]) {
                            float px = point.x + offsets[i].x;
                            float py = point.y + offsets[i].y;
                            if (px >= 0 && px < map_temp_.cols && py >= 0 && py < map_temp_.rows) {
                                sum += map_temp_.at<uchar>(py, px);
                            }
                        }
                        if (sum > max_sum) {
                            max_sum = sum;
                            best_dx = offsets[i].x;
                            best_dy = offsets[i].y;
                            best_dyaw = yaw_offsets[j];
                        }
                    }
                }

                lidar_x_ += best_dx;
                lidar_y_ += best_dy;
                lidar_yaw_ += best_dyaw;

                if(check(lidar_x_, lidar_y_, lidar_yaw_)) {
                    break;
                }
            } else {
                break;
            }
        }

        if(clear_countdown_ > -1) clear_countdown_--;
        if(clear_countdown_ == 0) {
            // ROS 2 service calls are asynchronous
            auto request = std::make_shared<std_srvs::srv::Empty::Request>();
            clear_costmaps_client_->async_send_request(request);
            RCLCPP_INFO(this->get_logger(), "Clearing costmaps.");
        }
    }

    bool check(float x, float y, float yaw)
    {
        const size_t max_size = 10;
        if(x == 0 && y == 0 && yaw == 0) {
            data_queue_.clear();
            return true;
        }

        data_queue_.push_back(std::make_tuple(x, y, yaw));
        if (data_queue_.size() > max_size) {
            data_queue_.pop_front();
        }

        if (data_queue_.size() == max_size) {
            auto& first = data_queue_.front();
            auto& last = data_queue_.back();
            float dx = std::abs(std::get<0>(last) - std::get<0>(first));
            float dy = std::abs(std::get<1>(last) - std::get<1>(first));
            float dyaw = std::abs(std::get<2>(last) - std::get<2>(first));

            if (dx < 5 && dy < 5 && dyaw < 5*deg_to_rad) {
                data_queue_.clear();
                return true;
            }
        }
        return false;
    }

    cv::Mat createGradientMask(int size)
    {
        cv::Mat mask(size, size, CV_8UC1);
        int center = size / 2;
        for (int y = 0; y < size; y++) {
            for (int x = 0; x < size; x++) {
                double distance = std::hypot(x - center, y - center);
                int value = cv::saturate_cast<uchar>(255 * std::max(0.0, 1.0 - distance / center));
                mask.at<uchar>(y, x) = value;
            }
        }
        return mask;
    }

    void processMap()
    {
        if (map_cropped_.empty()) return;

        map_temp_ = cv::Mat::zeros(map_cropped_.size(), CV_8UC1);
        cv::Mat gradient_mask = createGradientMask(101);

        for (int y = 0; y < map_cropped_.rows; y++) {
            for (int x = 0; x < map_cropped_.cols; x++) {
                if (map_cropped_.at<uchar>(y, x) == 100) {
                    int left = std::max(0, x - 50);
                    int top = std::max(0, y - 50);
                    int right = std::min(map_cropped_.cols - 1, x + 50);
                    int bottom = std::min(map_cropped_.rows - 1, y + 50);
                    cv::Rect roi(left, top, right - left + 1, bottom - top + 1);
                    cv::Mat region = map_temp_(roi);

                    int mask_left = 50 - (x - left);
                    int mask_top = 50 - (y - top);
                    cv::Rect mask_roi(mask_left, mask_top, roi.width, roi.height);
                    cv::Mat mask = gradient_mask(mask_roi);
                    cv::max(region, mask, region);
                }
            }
        }
    }

    // This function is called by the timer
    void pose_tf()
    {
        if (scan_count_ == 0) return;
        if (map_cropped_.empty() || map_msg_.data.empty() || map_msg_.info.resolution <= 0) return;

        double full_map_pixel_x = lidar_x_ + map_roi_info_.x_offset;
        double full_map_pixel_y = lidar_y_ + map_roi_info_.y_offset;

        double x_in_map_frame = full_map_pixel_x * map_msg_.info.resolution + map_msg_.info.origin.position.x;
        double y_in_map_frame = full_map_pixel_y * map_msg_.info.resolution + map_msg_.info.origin.position.y;
        double yaw_in_map_frame = -lidar_yaw_;

        tf2::Transform map_to_base;
        map_to_base.setOrigin(tf2::Vector3(x_in_map_frame, y_in_map_frame, 0.0));
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_in_map_frame);
        map_to_base.setRotation(q);

        geometry_msgs::msg::TransformStamped odom_to_base_msg;
        try {
            odom_to_base_msg = tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Cannot get transform from '%s' to '%s': %s",
                odom_frame_.c_str(), base_frame_.c_str(), ex.what());
            return;
        }

        tf2::Transform odom_to_base_tf2;
        tf2::fromMsg(odom_to_base_msg.transform, odom_to_base_tf2);
        tf2::Transform map_to_odom = map_to_base * odom_to_base_tf2.inverse();
        
        geometry_msgs::msg::TransformStamped map_to_odom_msg;
        // Use the node's clock to get the current time
        // map_to_odom_msg.header.stamp = this->get_clock()->now();
        map_to_odom_msg.header.stamp = odom_to_base_msg.header.stamp; 
        map_to_odom_msg.header.frame_id = map_frame_;
        map_to_odom_msg.child_frame_id = odom_frame_;
        map_to_odom_msg.transform = tf2::toMsg(map_to_odom);
        
        tf_broadcaster_->sendTransform(map_to_odom_msg);
    }
};

// Main function for ROS 2
int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarLocalizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}