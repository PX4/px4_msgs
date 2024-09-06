#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "vehicle_attitude_v2.h"
#include "vehicle_attitude_v3.h"
#include "ros_translations.h"

using namespace std::chrono_literals;

class TranslationNode : public rclcpp::Node
{
public:
	TranslationNode() : Node("translation_node")
	{
		_ros_translations = std::make_unique<RosTranslations>(*this, RegisteredTranslations::instance().translations());


		// Monitor subscriptions & publishers
		// TODO: event-based
		_node_update_timer = create_wall_timer(1s, [this](){
			std::vector<RosTranslations::TopicInfo> topic_info;
			const auto topics = get_topic_names_and_types();
			for (const auto& [topic_name, topic_types] : topics) {
				auto publishers = get_publishers_info_by_topic(topic_name);
				auto subscribers = get_subscriptions_info_by_topic(topic_name);
				// Filter out self
				int num_publishers = 0;
				for (const auto& publisher : publishers) {
					num_publishers += publisher.node_name() != this->get_name();
				}
				int num_subscribers = 0;
				for (const auto& subscriber : subscribers) {
					num_subscribers += subscriber.node_name() != this->get_name();
				}

				if (num_subscribers > 0 || num_publishers > 0) {
					topic_info.emplace_back(RosTranslations::TopicInfo{topic_name, num_subscribers, num_publishers});
				}
			}
			_ros_translations->updateCurrentTopics(topic_info);
		});
	}

private:
	std::unique_ptr<RosTranslations> _ros_translations;
	rclcpp::TimerBase::SharedPtr _node_update_timer;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TranslationNode>());
	rclcpp::shutdown();
	return 0;
}