#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    rclcpp::SensorDataQoS depth_qos;
    depth_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera/depth/image_rect_raw", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    RCLCPP_INFO(
      this->get_logger(), "Received depth image, ts: %.9d.%.9d", msg->header.stamp.sec,
      msg->header.stamp.nanosec);
  }
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
