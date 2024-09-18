/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

// Translate VehicleAttitude v1 <--> v2
#include <px4_msgs_old/msg/vehicle_attitude_v1.hpp>
#include <px4_msgs_old/msg/vehicle_attitude_v2.hpp>

#include "translation_util.h"
#include <cstring>

class VehicleAttitudeV2Translation {
public:
	using MessageOlder = px4_msgs_old::msg::VehicleAttitudeV1;
	static_assert(MessageOlder::MESSAGE_VERSION == 1);

	using MessageNewer = px4_msgs_old::msg::VehicleAttitudeV2;
	static_assert(MessageNewer::MESSAGE_VERSION == 2);

	static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {
		// No change in message definition
		static_assert(sizeof(msg_newer) == sizeof(msg_older));
		memcpy(&msg_newer, &msg_older, sizeof(msg_newer));
	}

	static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {
		memcpy(&msg_older, &msg_newer, sizeof(msg_newer));
	}

private:
};

REGISTER_MESSAGE_TRANSLATION_DIRECT("fmu/out/vehicle_attitude", VehicleAttitudeV2Translation);