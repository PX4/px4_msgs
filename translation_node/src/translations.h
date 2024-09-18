/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <string>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <functional>
#include <vector>
#include <memory>

#include "util.h"
#include "graph.h"

#include <rclcpp/rclcpp.hpp>


using TranslationCB = std::function<void(const std::vector<MessageBuffer>&, std::vector<MessageBuffer>&)>;
using SubscriptionFactoryCB = std::function<rclcpp::SubscriptionBase::SharedPtr(rclcpp::Node&, const std::function<void()>& on_topic_cb)>;
using PublicationFactoryCB = std::function<rclcpp::PublisherBase::SharedPtr(rclcpp::Node&)>;

struct Topic {
	std::string topic_name;
	MessageVersionType version{};

	SubscriptionFactoryCB subscription_factory;
	PublicationFactoryCB publication_factory;

	std::shared_ptr<void> message_buffer;
	size_t max_serialized_message_size{};
};

struct Translation {
	TranslationCB cb;
	std::vector<MessageIdentifier> inputs;
	std::vector<MessageIdentifier> outputs;
};

class Translations {
public:
	Translations() = default;

	void addTopic(Topic topic) { _topics.push_back(std::move(topic)); }
	void addTranslation(Translation translation) { _translations.push_back(std::move(translation)); }

	const std::vector<Topic>& topics() const { return _topics; }
	const std::vector<Translation>& translations() const { return _translations; }
private:
	std::vector<Topic> _topics;
	std::vector<Translation> _translations;
};
