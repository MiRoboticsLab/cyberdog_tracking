#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "protocol/msg/body_info.hpp"
#include "protocol/srv/body_region.hpp"

using std::placeholders::_1;

int loss_count_ = 0;

class SetObject : public rclcpp::Node
{
public:
  SetObject()
  : Node("set_object"), is_first(true)
  {
    rclcpp::SensorDataQoS sub_qos;
    sub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    subscription_ = this->create_subscription<protocol::msg::BodyInfo>(
      "body", sub_qos, std::bind(&SetObject::topic_callback, this, _1));

    reid_client_ = create_client<protocol::srv::BodyRegion>("tracking_object");
  }

private:
  bool call_service(
    rclcpp::Client<protocol::srv::BodyRegion>::SharedPtr & client,
    const sensor_msgs::msg::RegionOfInterest & roi)
  {
    auto req = std::make_shared<protocol::srv::BodyRegion::Request>();
    req->roi = roi;

    std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1);
    while (!client->wait_for_service(timeout)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    auto client_cb = [timeout](rclcpp::Client<protocol::srv::BodyRegion>::SharedFuture future) {
        std::future_status status = future.wait_for(timeout);

        if (status == std::future_status::ready) {
          if (0 != future.get()->success) {
            return false;
          } else {
            return true;
          }
        } else {
          return false;
        }
      };

    auto result = client->async_send_request(req, client_cb);
    return true;
  }

  int set_tracker(const protocol::msg::Body & body)
  {
    if (!call_service(reid_client_, body.roi)) {
      RCLCPP_ERROR(this->get_logger(), "Call reid service fail. ");
      return -1;
    }
    RCLCPP_INFO(this->get_logger(), "Call reid service success. ");
    return 0;
  }

  void topic_callback(const protocol::msg::BodyInfo::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received body info. ");
    bool is_reid = false;
    for (size_t i = 0; i < msg->count; ++i) {
      if (!msg->infos[i].reid.empty()) {
        is_reid = true;
        loss_count_ = 0;
      }
    }

    if (is_first) {
      if (msg->count > 0) {
        if (0 != set_tracker(msg->infos[0])) {
          RCLCPP_INFO(this->get_logger(), "Set first tracker fail. ");
        } else {
          is_first = false;
        }
      }
    } else {
      if (!is_reid) {
        loss_count_++;
        if (loss_count_ > 300 && msg->count > 0) {
          if (0 != set_tracker(msg->infos[0])) {
            RCLCPP_INFO(this->get_logger(), "Set first tracker fail. ");
          }
        }
      }
    }
  }

private:
  rclcpp::Subscription<protocol::msg::BodyInfo>::SharedPtr subscription_;
  rclcpp::Client<protocol::srv::BodyRegion>::SharedPtr reid_client_;

  bool is_first;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetObject>());
  rclcpp::shutdown();
  return 0;
}
