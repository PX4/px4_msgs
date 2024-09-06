#include "ros_translations.h"

static std::string getFullTopicName(const std::string& namespace_name, const std::string& topic_name) {
	std::string full_topic_name = topic_name;
	if (!full_topic_name.empty() && full_topic_name[0] != '/') {
		if (namespace_name.empty() || namespace_name.back() != '/') {
			full_topic_name = '/' + full_topic_name;
		}
		full_topic_name = namespace_name + full_topic_name;
	}
	return full_topic_name;
}

RosTranslationForTopic::RosTranslationForTopic(rclcpp::Node& node, std::string topic_name, const TranslationForTopic &translation)
	: _node(node), _topic_name(std::move(topic_name)) {
	assert(!translation.directTranslations().empty());
	// Find oldest version
	auto version_iter = std::min_element(translation.directTranslations().begin(), translation.directTranslations().end(),
												[](auto && a, auto&& b) {
		return a.newer.version < b.newer.version;
	});
	// Build version chain, use the newer element as the value for the node
	// So we need to add the older element for the lowest version first
	{
		VersionEntry entry{version_iter->older};
		_versions.push_back(std::move(entry));
	}

	while (version_iter != translation.directTranslations().end()) {
		// Add current version
		VersionEntry entry{version_iter->newer};
		entry.translation_cb_to_older = version_iter->translation_cb_to_older;
		entry.translation_cb_from_older = version_iter->translation_cb_from_older;
		_versions.push_back(std::move(entry));

		// Find next version (assumes there is no cycle in the definitions, otherwise there's an endless loop)
		version_iter = std::find_if(translation.directTranslations().begin(), translation.directTranslations().end(),
									[version_iter](auto&& a) {
			return version_iter->newer.version == a.older.version;
		});
	}

	if (_versions.size() != translation.directTranslations().size() + 1) {
		// This means there is a gap in the versions
		throw std::runtime_error(std::string("non-continuous versions for topic") + _topic_name);
	}

	handleLargestTopic();
}

void RosTranslationForTopic::updateSubsAndPubs() {
	const bool has_subscriber = std::any_of(_versions.begin(), _versions.end(), [](auto&& a) {
		return a.has_external_subscriber;
	});
	const bool has_publisher = std::any_of(_versions.begin(), _versions.end(), [](auto&& a) {
		return a.has_external_publisher;
	});
	if (has_subscriber && has_publisher) {
		// TODO: do not add anything if there's only a subscriber & publisher for a specific version

		for (unsigned index = 0; index < _versions.size(); ++index) {
			auto& version = _versions[index];
			// Has subscriber(s)?
			if (version.has_external_subscriber && !version.publication) {
				RCLCPP_INFO(_node.get_logger(), "Found subscriber for topic '%s', version: %i, adding publisher", _topic_name.c_str(), version.version);
				version.publication = version.publication_factory(_node);
			} else if (!version.has_external_subscriber && version.publication) {
				RCLCPP_INFO(_node.get_logger(), "No subscribers for topic '%s', version: %i, removing publisher", _topic_name.c_str(), version.version);
				version.publication.reset();
			}
			// Has publisher(s)?
			if (version.has_external_publisher && !version.subscription) {
				RCLCPP_INFO(_node.get_logger(), "Found publisher for topic '%s', version: %i, adding subscriber", _topic_name.c_str(), version.version);
				version.subscription = version.subscription_factory(_node, [this, index](void* data) {
					onSubscriptionUpdated(index, data);
				});
			} else if (!version.has_external_publisher && version.subscription) {
				RCLCPP_INFO(_node.get_logger(), "No publishers for topic '%s', version: %i, removing subscriber", _topic_name.c_str(), version.version);
				version.subscription.reset();
			}

		}
	} else {
		// Clear all
		for (auto& version : _versions) {
			version.publication.reset();
			version.subscription.reset();
		}
	}
}

void RosTranslationForTopic::onSubscriptionUpdated(unsigned version_index, void *data) {
	const auto& entry = _versions[version_index];
	const auto lowest_publisher_iter = std::find_if(_versions.begin(), _versions.end(), [](auto&& a) { return a.publication != nullptr; });
	// Convert to lower versions
	if (lowest_publisher_iter != _versions.end()) {
		const unsigned lowest_index = std::distance(_versions.begin(), lowest_publisher_iter);
		void* current_data = data;
		for (unsigned index = version_index; index > lowest_index; --index) {
			// Convert message
			_versions[index].translation_cb_to_older(current_data, _versions[index-1].message_buffer.get());
			current_data = _versions[index-1].message_buffer.get();
			// Publish if there is a publisher
			if (_versions[index-1].publication) {
				rcl_publish(_versions[index-1].publication->get_publisher_handle().get(), current_data, nullptr);
			}
		}
	}
	// Convert to higher versions
	const auto highest_publisher_iter = std::find_if(_versions.rbegin(), _versions.rend(), [](auto&& a) { return a.publication != nullptr; });
	if (highest_publisher_iter != _versions.rend()) {
		const unsigned highest_index = std::distance(highest_publisher_iter, _versions.rend()) - 1;
		void* current_data = data;
		for (unsigned index = version_index; index < highest_index; ++index) {
			// Convert message
			_versions[index + 1].translation_cb_from_older(current_data, _versions[index+1].message_buffer.get());
			current_data = _versions[index+1].message_buffer.get();
			// Publish if there is a publisher
			if (_versions[index+1].publication) {
				rcl_publish(_versions[index+1].publication->get_publisher_handle().get(), current_data, nullptr);
			}
		}
	}
}

void RosTranslationForTopic::handleLargestTopic() {
	// FastDDS caches some type information per DDS participant when first creating a publisher or subscriber for a given
	// type. The information that is relevant for us is the maximum serialized message size.
	// Since different versions can have different sizes, we need to ensure the first publication or subscription
	// happens with the version of the largest size. Otherwise, an out-of-memory exception can be triggered.
	// And the type must continue to be in use (so we cannot delete it)
	auto version_iter = std::max_element(_versions.begin(), _versions.end(),
										 [](auto && a, auto&& b) {
											 return a.max_serialized_message_size < b.max_serialized_message_size;
										 });
	_publisher_largest_topic = version_iter->publication_factory(_node);
}

RosTranslations::RosTranslations(rclcpp::Node &node, const Translations &translations)
		: _node(node) {

	for (const auto& [topic_name, translation]: translations.topicTranslations()) {
		const std::string full_topic_name = getFullTopicName(node.get_effective_namespace(), topic_name);
		auto [iter, _] = _topics.emplace(full_topic_name, RosTranslationForTopic{_node, full_topic_name, translation});

		// Print versions info
		const std::string versions = std::accumulate(std::next(iter->second.versions().begin()), iter->second.versions().end(),
										std::to_string(iter->second.versions()[0].version), // start with first element
										[](std::string a, auto&& b) {
			return std::move(a) + ", " + std::to_string(b.version);
		});
		RCLCPP_INFO(_node.get_logger(), "Versions for topic '%s': %s", iter->second.topicName().c_str(), versions.c_str());
	}
}

void RosTranslations::updateCurrentTopics(const std::vector<TopicInfo> &topics) {
	for (auto& [topic_name, translation] : _topics) {
		translation.resetExternalSubPub();
	}
	for (const auto& topic_info : topics) {
		const auto [non_versioned_topic_name, version] = getNonVersionedTopicName(topic_info.topic_name);
		auto iter = _topics.find(non_versioned_topic_name);
		if (iter == _topics.end()) {
			continue;
		}
		// It's a topic we're interested in, find the version
		bool found_version = false;
		for (auto& entry : iter->second.versions()) {
			if (entry.version == version) {
				found_version = true;
				if (topic_info.num_publishers > 0) {
					entry.has_external_publisher = true;
				}
				if (topic_info.num_subscribers > 0) {
					entry.has_external_subscriber = true;
				}
			}
		}
		if (!found_version && !iter->second.getAndSetErrorPrinted()) {
			RCLCPP_WARN(_node.get_logger(), "Unsupported version for topic '%s': %i", non_versioned_topic_name.c_str(), version);
		}
	}

	// Now update the subscriptions / publishers depending on what we found
	for (auto& [topic_name, translation] : _topics) {
		translation.updateSubsAndPubs();
	}
}

