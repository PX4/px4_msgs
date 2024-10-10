/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "vehicle_attitude_v2.h"
#include "vehicle_attitude_v3.h"
#include "vehicle_local_global_position_v2.h"
#include "pub_sub_graph.h"
#include "monitor.h"

using namespace std::chrono_literals;

class RosTranslationNode : public rclcpp::Node
{
public:
	RosTranslationNode() : Node("translation_node")
	{
		_pub_sub_graph = std::make_unique<PubSubGraph>(*this, RegisteredTranslations::instance().translations());
		_monitor = std::make_unique<Monitor>(*this, *_pub_sub_graph);
	}

private:
	std::unique_ptr<PubSubGraph> _pub_sub_graph;
	rclcpp::TimerBase::SharedPtr _node_update_timer;
	std::unique_ptr<Monitor> _monitor;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RosTranslationNode>());
	rclcpp::shutdown();
	return 0;
}