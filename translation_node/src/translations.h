#pragma once

#include <string>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <functional>
#include <vector>
#include <memory>

#include "util.h"

#include <rclcpp/rclcpp.hpp>


using DirectTranslationCB = std::function<void(const void*, void*)>;
using SubscriptionFactoryCB = std::function<rclcpp::SubscriptionBase::SharedPtr(rclcpp::Node&, const std::function<void(void*)>& on_topic_cb)>;
using PublicationFactoryCB = std::function<rclcpp::PublisherBase::SharedPtr(rclcpp::Node&)>;

struct DirectTranslationData {
	struct Version {
		MessageVersionType version;
		std::shared_ptr<void> message_buffer;
		SubscriptionFactoryCB subscription_factory;
		PublicationFactoryCB publication_factory;
		size_t max_serialized_message_size{};
	};

	Version older;
	Version newer;

	DirectTranslationCB translation_cb_from_older;
	DirectTranslationCB translation_cb_to_older;
};

class TranslationForTopic {
public:
	explicit TranslationForTopic(std::string topic_name="") : _topic_name(std::move(topic_name)) {}

	void registerVersion(DirectTranslationData data);

	const std::string& topicName() const { return _topic_name; }
	const std::vector<DirectTranslationData>& directTranslations() const { return _direct_translations; };

private:
	const std::string _topic_name;
	std::vector<DirectTranslationData> _direct_translations;
};

class Translations {
public:

	Translations() = default;

	void registerDirectTranslation(const std::string& topic_name, DirectTranslationData data);

	const std::unordered_map<std::string, TranslationForTopic>& topicTranslations() const { return _topic_translations; }

private:
	std::unordered_map<std::string, TranslationForTopic> _topic_translations;
};
