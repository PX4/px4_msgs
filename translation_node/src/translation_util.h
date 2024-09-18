/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include "translations.h"
#include "util.h"

#include <rosidl_typesupport_cpp/message_type_support_dispatch.hpp>
#include <rosidl_typesupport_fastrtps_cpp/message_type_support.h>

class RegisteredTranslations {
public:

	RegisteredTranslations(RegisteredTranslations const&) = delete;
	void operator=(RegisteredTranslations const&) = delete;


	static RegisteredTranslations& instance() {
		static RegisteredTranslations instance;
		return instance;
	}

	template<class T>
	void registerDirectTranslation(const std::string& topic_name) {
		_translations.addTopic(getTopicForMessageType<typename T::MessageOlder>(topic_name));
		_translations.addTopic(getTopicForMessageType<typename T::MessageNewer>(topic_name));

		// Translation callbacks
		auto translation_cb_from_older = [](const std::vector<MessageBuffer>& older_msg, std::vector<MessageBuffer>& newer_msg) {
			T::fromOlder(*(const typename T::MessageOlder*)older_msg[0].get(), *(typename T::MessageNewer*)newer_msg[0].get());
		};
		auto translation_cb_to_older = [](const std::vector<MessageBuffer>& newer_msg, std::vector<MessageBuffer>& older_msg) {
			T::toOlder(*(const typename T::MessageNewer*)newer_msg[0].get(), *(typename T::MessageOlder*)older_msg[0].get());
		};
		_translations.addTranslation({translation_cb_from_older,
									  {MessageIdentifier{topic_name, T::MessageOlder::MESSAGE_VERSION}},
									  {MessageIdentifier{topic_name, T::MessageNewer::MESSAGE_VERSION}}});
		_translations.addTranslation({translation_cb_to_older,
									  {MessageIdentifier{topic_name, T::MessageNewer::MESSAGE_VERSION}},
									  {MessageIdentifier{topic_name, T::MessageOlder::MESSAGE_VERSION}}});
	}

	const Translations& translations() const { return _translations; }

private:
	RegisteredTranslations() = default;

	template<typename RosMessageType>
	Topic getTopicForMessageType(const std::string& topic_name) {
		Topic ret{};
		ret.topic_name = topic_name;
		ret.version = RosMessageType::MESSAGE_VERSION;
		auto message_buffer = std::make_shared<RosMessageType>();
		ret.message_buffer = std::static_pointer_cast<void>(message_buffer);

		// Subscription/Publication factory methods
		const std::string topic_name_versioned = getVersionedTopicName(topic_name, ret.version);
		ret.subscription_factory = [topic_name_versioned, message_buffer](rclcpp::Node& node,
												const std::function<void()>& on_topic_cb) -> rclcpp::SubscriptionBase::SharedPtr {
			return std::dynamic_pointer_cast<rclcpp::SubscriptionBase>(
					node.create_subscription<RosMessageType>(topic_name_versioned, rclcpp::QoS(1).best_effort(),
															 [on_topic_cb=on_topic_cb, message_buffer](typename RosMessageType::UniquePtr msg) -> void {
																 *message_buffer = *msg;
																 on_topic_cb();
															 }));
		};
		ret.publication_factory = [topic_name_versioned](rclcpp::Node& node) -> rclcpp::PublisherBase::SharedPtr {
			return std::dynamic_pointer_cast<rclcpp::PublisherBase>(
					node.create_publisher<RosMessageType>(topic_name_versioned, rclcpp::QoS(1).best_effort()));
		};

		const auto type_handle = rclcpp::get_message_type_support_handle<RosMessageType>();
		const auto fastrtps_handle = rosidl_typesupport_cpp::get_message_typesupport_handle_function(&type_handle, "rosidl_typesupport_fastrtps_cpp");
		if (fastrtps_handle) {
			const auto *callbacks = static_cast<const message_type_support_callbacks_t *>(fastrtps_handle->data);
			char bound_info;
			ret.max_serialized_message_size = callbacks->max_serialized_size(bound_info);
		}

		return ret;
	}

	Translations _translations;
};

template<class T>
class RegistrationHelperDirect {
public:
	explicit RegistrationHelperDirect(const std::string& topic_name) {
		RegisteredTranslations::instance().registerDirectTranslation<T>(topic_name);
	}
	RegistrationHelperDirect(RegistrationHelperDirect const&) = delete;
	void operator=(RegistrationHelperDirect const&) = delete;
private:
};

#define REGISTER_MESSAGE_TRANSLATION_DIRECT(topic_name, class_name) \
	static RegistrationHelperDirect<class_name> class_name##_registration(topic_name);
