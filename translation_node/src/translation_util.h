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
	void registerTranslation(const std::string& topic_name) {
		DirectTranslationData data{};
		data.older = getVersionForMessageType<typename T::MessageOlder>(topic_name);
		data.newer = getVersionForMessageType<typename T::MessageNewer>(topic_name);

		// Translation callbacks
		data.translation_cb_from_older = [](const void* older_msg, void* newer_msg) {
			T::fromOlder(*(const typename T::MessageOlder*)older_msg, *(typename T::MessageNewer*)newer_msg);
		};
		data.translation_cb_to_older = [](const void* newer_msg, void* older_msg) {
			T::toOlder(*(const typename T::MessageNewer*)newer_msg, *(typename T::MessageOlder*)older_msg);
		};

		_translations.registerDirectTranslation(topic_name, std::move(data));
	}

	const Translations& translations() const { return _translations; }

private:
	RegisteredTranslations() = default;

	template<typename RosMessageType>
	DirectTranslationData::Version getVersionForMessageType(const std::string& topic_name) {
		DirectTranslationData::Version ret{};
		ret.version = RosMessageType::MESSAGE_VERSION;
		ret.message_buffer = std::static_pointer_cast<void>(std::make_shared<RosMessageType>());

		// Subscription/Publication factory methods
		const std::string topic_name_versioned = getVersionedTopicName(topic_name, ret.version);
		ret.subscription_factory = [topic_name_versioned](rclcpp::Node& node,
												const std::function<void(void*)>& on_topic_cb) -> rclcpp::SubscriptionBase::SharedPtr {
			return std::dynamic_pointer_cast<rclcpp::SubscriptionBase>(
					node.create_subscription<RosMessageType>(topic_name_versioned, rclcpp::QoS(1).best_effort(),
															 [on_topic_cb=on_topic_cb](typename RosMessageType::UniquePtr msg) -> void {
																 on_topic_cb(msg.get());
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
class RegistrationHelper {
public:
	explicit RegistrationHelper(const std::string& topic_name) {
		RegisteredTranslations::instance().registerTranslation<T>(topic_name);
	}
	RegistrationHelper(RegistrationHelper const&) = delete;
	void operator=(RegistrationHelper const&) = delete;
private:
};

#define REGISTER_MESSAGE_TRANSLATION_DIRECT(topic_name, class_name) \
	static RegistrationHelper<class_name> class_name##_registration(topic_name);
