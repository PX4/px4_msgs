/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

// Translate VehicleAttitude v2 <--> v3
#include <px4_msgs_old/msg/vehicle_attitude_v2.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>

#include "translation_util.h"

class VehicleAttitudeV3Translation {
public:
	using MessageOlder = px4_msgs_old::msg::VehicleAttitudeV2;
	static_assert(MessageOlder::MESSAGE_VERSION == 2);

	using MessageNewer = px4_msgs::msg::VehicleAttitude;
	static_assert(MessageNewer::MESSAGE_VERSION == 3);

	static constexpr const char* kTopic = "fmu/out/vehicle_attitude";

	static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {
		msg_newer.timestamp = msg_older.timestamp;
		msg_newer.timestamp_sample = msg_older.timestamp_sample;
		msg_newer.q[0] = msg_older.q[0];
		msg_newer.q[1] = msg_older.q[1];
		msg_newer.q[2] = msg_older.q[2];
		msg_newer.q[3] = msg_older.q[3];
		msg_newer.delta_q_reset = msg_older.delta_q_reset;
		msg_newer.quat_reset_counter = msg_older.quat_reset_counter;

		msg_newer.new_field = 34;
	}

	static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {
		msg_older.timestamp = msg_newer.timestamp;
		msg_older.timestamp_sample = msg_newer.timestamp_sample;
		msg_older.q[0] = msg_newer.q[0];
		msg_older.q[1] = msg_newer.q[1];
		msg_older.q[2] = msg_newer.q[2];
		msg_older.q[3] = msg_newer.q[3];
		msg_older.delta_q_reset = msg_newer.delta_q_reset;
		msg_older.quat_reset_counter = msg_newer.quat_reset_counter;
	}

private:
};

REGISTER_TOPIC_TRANSLATION_DIRECT(VehicleAttitudeV3Translation);