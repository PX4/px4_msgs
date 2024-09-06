#pragma once

#include <rclcpp/rclcpp.hpp>
#include "translations.h"

class RosTranslationVersion {
public:
private:
	rclcpp::SubscriptionBase::SharedPtr _subscriber;
	rclcpp::PublisherBase::SharedPtr _publisher;
};

class RosTranslationForTopic {
public:
	struct VersionEntry {
		VersionEntry() = default;
		explicit VersionEntry(const DirectTranslationData::Version& version)
		: version(version.version), message_buffer(version.message_buffer),
		max_serialized_message_size(version.max_serialized_message_size),
		subscription_factory(version.subscription_factory), publication_factory(version.publication_factory) {}

		MessageVersionType version; ///< corresponds to the 'newer' version in the translation
		DirectTranslationCB translation_cb_from_older;
		DirectTranslationCB translation_cb_to_older;
		std::shared_ptr<void> message_buffer;
		size_t max_serialized_message_size{};

		SubscriptionFactoryCB subscription_factory;
		PublicationFactoryCB publication_factory;

		// Keep track if there's currently a publisher/subscriber
		bool has_external_publisher{false};
		bool has_external_subscriber{false};

		rclcpp::SubscriptionBase::SharedPtr subscription;
		rclcpp::PublisherBase::SharedPtr publication;
	};
	explicit RosTranslationForTopic(rclcpp::Node& node, std::string topic_name, const TranslationForTopic& translation);

	const std::string& topicName() const { return _topic_name; }
	std::vector<VersionEntry>& versions() { return _versions; }

	void resetExternalSubPub() {
		for (auto& entry : _versions) {
			entry.has_external_publisher = false;
			entry.has_external_subscriber = false;
		}
	}

	void updateSubsAndPubs();

	bool getAndSetErrorPrinted() {
		const bool error_printed = _error_printed;
		_error_printed = true;
		return error_printed;
	}

private:
	void onSubscriptionUpdated(unsigned version_index, void* data);
	void handleLargestTopic();

	rclcpp::Node& _node;
	const std::string _topic_name;
	std::vector<VersionEntry> _versions;
	bool _error_printed{false}; ///< ensure errors are printed only once

	std::shared_ptr<rclcpp::PublisherBase> _publisher_largest_topic;
};

class RosTranslations {
public:
	struct TopicInfo {
		std::string topic_name; ///< fully qualified topic name (with namespace)
		int num_subscribers; ///< does not include this node's subscribers
		int num_publishers; ///< does not include this node's publishers
	};

	explicit RosTranslations(rclcpp::Node& node, const Translations& translations);

	void updateCurrentTopics(const std::vector<TopicInfo>& topics);
private:
	rclcpp::Node& _node;
	std::unordered_map<std::string, RosTranslationForTopic> _topics;
};