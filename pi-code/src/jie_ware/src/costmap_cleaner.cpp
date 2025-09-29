#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_srvs/srv/empty.hpp>

class CostmapClearer : public rclcpp::Node
{
public:
    CostmapClearer()
    : Node("costmap_cleaner")
    {
        // Subscribe to /initialpose
        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&CostmapClearer::initialPoseCallback, this, std::placeholders::_1)
        );

        // Create service client for clearing costmaps
        clear_costmaps_client_ =
            this->create_client<std_srvs::srv::Empty>("/clear_costmaps");
            // In Nav2, the service is usually /clear_costmaps, not /move_base/clear_costmaps
    }

private:
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        (void)msg; // suppress unused warning if we don’t need msg

        if (!clear_costmaps_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Clear costmaps service not available.");
            return;
        }

        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto future_result = clear_costmaps_client_->async_send_request(request);

        // Optionally wait for result (not strictly required for Empty)
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result)
            == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "成功清除代价地图！");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "代价地图清除失败.");
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_costmaps_client_;
};

int main(int argc, char **argv)
{
    setlocale(LC_ALL,""); // Keep Chinese logging intact
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CostmapClearer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
