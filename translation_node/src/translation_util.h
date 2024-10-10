/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include "translations.h"
#include "util.h"
#include "template_util.h"

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

	/**
	 * @brief Register a translation class with 1 input and 1 output message.
	 *
	 * The translation class has the form:
	 *
	 * ```
	 * class MyTranslation {
	 * public:
	 * 	using MessageOlder = px4_msgs_old::msg::VehicleAttitudeV2;
	 *
	 * 	using MessageNewer = px4_msgs::msg::VehicleAttitude;
	 *
	 * 	static constexpr const char* kTopic = "fmu/out/vehicle_attitude";
	 *
	 * 	static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {
	 * 	    // set msg_newer from msg_older
	 * 	}
	 *
	 * 	static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {
	 * 	    // set msg_older from msg_newer
	 * 	}
	 * };
	 * ```
	 */
	template<class T>
	void registerDirectTranslation() {
		const std::string topic_name = T::kTopic;
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

	/**
	 * @brief Register a translation class with N input and M output messages.
	 *
	 * The translation class has the form:
	 * ```
	 * class MyTranslation {
	 * public:
	 * 	using MessagesOlder = TypesArray<ROS_MSG_OLDER_1, ROS_MSG_OLDER_2, ...>;
	 * 	static constexpr const char* kTopicsOlder[] = {
	 * 			"fmu/out/vehicle_global_position",
	 * 			"fmu/out/vehicle_local_position",
	 * 			...
	 * 	};
	 *
	 * 	using MessagesNewer = TypesArray<ROS_MSG_NEWER_1, ROS_MSG_NEWER_2, ...>;
	 * 	static constexpr const char* kTopicsNewer[] = {
	 * 			"fmu/out/vehicle_global_position",
	 * 			"fmu/out/vehicle_local_position",
	 * 			...
	 * 	};
	 *
	 * 	static void fromOlder(const MessagesOlder::Type1 &msg_older1, const MessagesOlder::Type2 &msg_older2, ...
	 * 						  MessagesNewer::Type1 &msg_newer1, MessagesNewer::Type2 &msg_newer2, ...) {
	 * 		// Set msg_newerX from msg_olderX
	 * 	}
	 *
	 * 	static void toOlder(const MessagesNewer::Type1 &msg_newer1, const MessagesNewer::Type2 &msg_newer2, ...
	 * 						MessagesOlder::Type1 &msg_older1, MessagesOlder::Type2 &msg_older2, ...) {
	 * 		// Set msg_olderX from msg_newerX
	 * 	}
	 * };
	 * ```
	 */
	template<class T>
	void registerTranslation() {
		const auto topics_older = getTopicsForMessageType(typename T::MessagesOlder::args(), T::kTopicsOlder);
		std::vector<MessageIdentifier> topics_older_identifiers;
		for (const auto& topic : topics_older) {
			_translations.addTopic(topic);
			topics_older_identifiers.emplace_back(topic.id);
		}
		const auto topics_newer = getTopicsForMessageType(typename T::MessagesNewer::args(),T::kTopicsNewer);
		std::vector<MessageIdentifier> topics_newer_identifiers;
		for (const auto& topic : topics_newer) {
			_translations.addTopic(topic);
			topics_newer_identifiers.emplace_back(topic.id);
		}

		// Translation callbacks
		const auto translation_cb_from_older = [](const std::vector<MessageBuffer>& older_msgs, std::vector<MessageBuffer>& newer_msgs) {
			call_translation_function(&T::fromOlder, typename T::MessagesOlder::args(), typename T::MessagesNewer::args(), older_msgs, newer_msgs);
		};
		const auto translation_cb_to_older = [](const std::vector<MessageBuffer>& newer_msgs, std::vector<MessageBuffer>& older_msgs) {
			call_translation_function(&T::toOlder, typename T::MessagesNewer::args(), typename T::MessagesOlder::args(), newer_msgs, older_msgs);
		};
		{
			// Older -> Newer
			Translation translation;
			translation.cb = translation_cb_from_older;
			translation.inputs = topics_older_identifiers;
			translation.outputs = topics_newer_identifiers;
			_translations.addTranslation(std::move(translation));
		}
		{
			// Newer -> Older
			Translation translation;
			translation.cb = translation_cb_to_older;
			translation.inputs = topics_newer_identifiers;
			translation.outputs = topics_older_identifiers;
			_translations.addTranslation(std::move(translation));
		}
	}

	const Translations& translations() const { return _translations; }

protected:
	RegisteredTranslations() = default;
private:

	template<typename RosMessageType>
	Topic getTopicForMessageType(const std::string& topic_name) {
		Topic ret{};
		ret.id.topic_name = topic_name;
		ret.id.version = RosMessageType::MESSAGE_VERSION;
		auto message_buffer = std::make_shared<RosMessageType>();
		ret.message_buffer = std::static_pointer_cast<void>(message_buffer);

		// Subscription/Publication factory methods
		const std::string topic_name_versioned = getVersionedTopicName(topic_name, ret.id.version);
		ret.subscription_factory = [topic_name_versioned, message_buffer](rclcpp::Node& node,
												const std::function<void()>& on_topic_cb) -> rclcpp::SubscriptionBase::SharedPtr {
			return std::dynamic_pointer_cast<rclcpp::SubscriptionBase>(
					// Note: template instantiation of subscriptions slows down compilation considerably, see
					// https://github.com/ros2/rclcpp/issues/1949
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

	template<typename... RosMessageTypes, size_t... Is>
	std::vector<Topic> getTopicsForMessageTypeImpl(const char* const topics[], std::integer_sequence<size_t, Is...>) {
		std::vector<Topic> ret {
			getTopicForMessageType<RosMessageTypes>(topics[Is])...
		};
		return ret;
	}

	template<typename... RosMessageTypes, size_t N>
	std::vector<Topic> getTopicsForMessageType(Pack<RosMessageTypes...>, const char* const (&topics)[N]) {
		static_assert(N == sizeof...(RosMessageTypes), "Number of topics does not match number of message types");
		return getTopicsForMessageTypeImpl<RosMessageTypes...>(topics, std::index_sequence_for<RosMessageTypes...>{});
	}

	Translations _translations;
};

template<class T>
class TopicRegistrationHelperDirect {
public:
	explicit TopicRegistrationHelperDirect(const char* dummy) {
		// There's something strange: when there is no argument passed, the
		// compiler removes the static object completely. I don't know
		// why but this dummy variable prevents that.
		(void)dummy;
		RegisteredTranslations::instance().registerDirectTranslation<T>();
	}
	TopicRegistrationHelperDirect(TopicRegistrationHelperDirect const&) = delete;
	void operator=(TopicRegistrationHelperDirect const&) = delete;
private:
};

#define REGISTER_TOPIC_TRANSLATION_DIRECT(class_name) \
	TopicRegistrationHelperDirect<class_name> class_name##_registration_direct("dummy");

template<class T>
class TopicRegistrationHelper {
public:
	explicit TopicRegistrationHelper(const char* dummy) {
		(void)dummy;
		RegisteredTranslations::instance().registerTranslation<T>();
	}
	TopicRegistrationHelper(TopicRegistrationHelper const&) = delete;
	void operator=(TopicRegistrationHelper const&) = delete;
private:
};

#define REGISTER_TOPIC_TRANSLATION(class_name) \
	TopicRegistrationHelper<class_name> class_name##_registration("dummy");
