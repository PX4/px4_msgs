/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include "monitor.h"
using namespace std::chrono_literals;

Monitor::Monitor(rclcpp::Node &node, PubSubGraph& pub_sub_graph)
	: _node(node), _pub_sub_graph(pub_sub_graph) {

	// Monitor subscriptions & publishers
	// TODO: event-based
	_node_update_timer = _node.create_wall_timer(1s, [this]() {
		updateNow();
	});
}

void Monitor::updateNow() {
	std::vector<PubSubGraph::TopicInfo> topic_info;
	const auto topics = _node.get_topic_names_and_types();
	for (const auto& [topic_name, topic_types] : topics) {
		auto publishers = _node.get_publishers_info_by_topic(topic_name);
		auto subscribers = _node.get_subscriptions_info_by_topic(topic_name);
		// Filter out self
		int num_publishers = 0;
		for (const auto& publisher : publishers) {
			num_publishers += publisher.node_name() != _node.get_name();
		}
		int num_subscribers = 0;
		for (const auto& subscriber : subscribers) {
			num_subscribers += subscriber.node_name() != _node.get_name();
		}

		if (num_subscribers > 0 || num_publishers > 0) {
			topic_info.emplace_back(PubSubGraph::TopicInfo{topic_name, num_subscribers, num_publishers});
		}
	}
	_pub_sub_graph.updateCurrentTopics(topic_info);
}
