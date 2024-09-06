#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"

template <typename T>
static std::string topic_version_suffix() {
	return "_v" + std::to_string(T::MESSAGE_VERSION);
}

// Explicit message version check
static_assert(px4_msgs::msg::VehicleAttitude::MESSAGE_VERSION == 3, "unexpected px4_msgs::VehicleAttitude version (new version?)");

class MinimalSubscriber : public rclcpp::Node
{
public:
	MinimalSubscriber()
			: Node("minimal_subscriber")
	{
		auto topic_callback =
				[this](px4_msgs::msg::VehicleAttitude::UniquePtr msg) -> void {
					RCLCPP_INFO(this->get_logger(), "Got: timestamp: '%lli', new_field: '%i'", msg->timestamp, msg->new_field);
				};
		subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
				"/fmu/out/vehicle_attitude" + topic_version_suffix<px4_msgs::msg::VehicleAttitude>(),
				rclcpp::QoS(1).best_effort(), topic_callback);
	}

private:
	rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MinimalSubscriber>());
	rclcpp::shutdown();
	return 0;
}