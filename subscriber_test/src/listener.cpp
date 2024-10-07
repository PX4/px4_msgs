#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"

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
		{
			auto topic_callback =
					[this](px4_msgs::msg::VehicleAttitude::UniquePtr msg) -> void {
						RCLCPP_INFO(this->get_logger(), "Got attitude: timestamp: '%lli', new_field: '%i'", msg->timestamp,
									(int) msg->new_field);
					};
			subscription_att_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
					"/fmu/out/vehicle_attitude" + topic_version_suffix<px4_msgs::msg::VehicleAttitude>(),
					rclcpp::QoS(1).best_effort(), topic_callback);
		}
		{
			auto topic_callback =
					[this](px4_msgs::msg::VehicleGlobalPosition::UniquePtr msg) -> void {
						RCLCPP_INFO(this->get_logger(), "Got global pos: timestamp: '%lli', alt: %.3f dist bottom: %.3f, eph: %.3f, terrain_alt: %.3f", msg->timestamp,
									(double)msg->alt, (double)msg->dist_bottom, (double)msg->eph, (double)msg->terrain_alt);
					};
			subscription_gpos_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
					"/fmu/out/vehicle_global_position" + topic_version_suffix<px4_msgs::msg::VehicleGlobalPosition>(),
					rclcpp::QoS(1).best_effort(), topic_callback);
		}
		{
			auto topic_callback =
					[this](px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) -> void {
						RCLCPP_INFO(this->get_logger(), "Got local pos: timestamp: '%lli', dist_bottom_var: %.3f, delta_alt: %.3f, alt_reset_counter: %i, delta_bottom: %.3f, eph: %.3f",
									msg->timestamp,
									(double)msg->dist_bottom_var, (double)msg->delta_alt, msg->alt_reset_counter, (double)msg->delta_dist_bottom, (double)msg->eph);
					};
			subscription_lpos_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
					"/fmu/out/vehicle_local_position" + topic_version_suffix<px4_msgs::msg::VehicleLocalPosition>(),
					rclcpp::QoS(1).best_effort(), topic_callback);
		}
	}

private:
	rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr subscription_att_;
	rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr subscription_gpos_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition >::SharedPtr subscription_lpos_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MinimalSubscriber>());
	rclcpp::shutdown();
	return 0;
}