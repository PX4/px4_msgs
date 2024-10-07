/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

// Translate VehicleGlobalPosition/VehicleLocalPosition v1 <--> v2
#include <px4_msgs_old/msg/vehicle_global_position_v1.hpp>
#include <px4_msgs_old/msg/vehicle_local_position_v1.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include "translation_util.h"

class VehicleLocalGlobalPositionV2Translation {
public:
	using MessagesOlder = TypesArray<px4_msgs_old::msg::VehicleGlobalPositionV1, px4_msgs_old::msg::VehicleLocalPositionV1>;
	static constexpr const char* kTopicsOlder[] = {
			"fmu/out/vehicle_global_position",
			"fmu/out/vehicle_local_position",
	};
	static_assert(px4_msgs_old::msg::VehicleGlobalPositionV1::MESSAGE_VERSION == 1);
	static_assert(px4_msgs_old::msg::VehicleLocalPositionV1::MESSAGE_VERSION == 1);

	using MessagesNewer = TypesArray<px4_msgs::msg::VehicleGlobalPosition, px4_msgs::msg::VehicleLocalPosition>;
	static constexpr const char* kTopicsNewer[] = {
			"fmu/out/vehicle_global_position",
			"fmu/out/vehicle_local_position",
	};
	static_assert(px4_msgs::msg::VehicleGlobalPosition::MESSAGE_VERSION == 2);
	static_assert(px4_msgs::msg::VehicleLocalPosition::MESSAGE_VERSION == 2);

	static void fromOlder(const MessagesOlder::Type1 &msg_older1, const MessagesOlder::Type2 &msg_older2,
						  MessagesNewer::Type1 &msg_newer1, MessagesNewer::Type2 &msg_newer2) {
		// Moved from Global to Local:
		msg_newer2.delta_alt = msg_older1.delta_alt;
		msg_newer2.delta_terrain = msg_older1.delta_terrain;
		msg_newer2.lat_lon_reset_counter = msg_older1.lat_lon_reset_counter;
		msg_newer2.alt_reset_counter = msg_older1.alt_reset_counter;
		msg_newer2.terrain_reset_counter = msg_older1.terrain_reset_counter;

		// Moved from Local to Global:
		msg_newer1.dist_bottom = msg_older2.dist_bottom;

		// Global position
		msg_newer1.timestamp = msg_older1.timestamp;
		msg_newer1.timestamp_sample = msg_older1.timestamp_sample;
		msg_newer1.lat = msg_older1.lat;
		msg_newer1.lon = msg_older1.lon;
		msg_newer1.alt = msg_older1.alt;
		msg_newer1.alt_ellipsoid = msg_older1.alt_ellipsoid;
		msg_newer1.eph = msg_older1.eph;
		msg_newer1.epv = msg_older1.epv;
		msg_newer1.terrain_alt = msg_older1.terrain_alt;
		msg_newer1.terrain_alt_valid = msg_older1.terrain_alt_valid;
		msg_newer1.dead_reckoning = msg_older1.dead_reckoning;

		// Local position
		msg_newer2.timestamp = msg_older2.timestamp;
		msg_newer2.timestamp_sample = msg_older2.timestamp_sample;
		msg_newer2.xy_valid = msg_older2.xy_valid;
		msg_newer2.z_valid = msg_older2.z_valid;
		msg_newer2.v_xy_valid = msg_older2.v_xy_valid;
		msg_newer2.v_z_valid = msg_older2.v_z_valid;
		msg_newer2.x = msg_older2.x;
		msg_newer2.y = msg_older2.y;
		msg_newer2.z = msg_older2.z;
		msg_newer2.delta_xy = msg_older2.delta_vxy;
		msg_newer2.xy_reset_counter = msg_older2.xy_reset_counter;
		msg_newer2.delta_z = msg_older2.delta_z;
		msg_newer2.z_reset_counter = msg_older2.z_reset_counter;
		msg_newer2.vx = msg_older2.vx;
		msg_newer2.vy = msg_older2.vy;
		msg_newer2.vz = msg_older2.vz;
		msg_newer2.z_deriv = msg_older2.z_deriv;
		msg_newer2.delta_vxy = msg_older2.delta_vxy;
		msg_newer2.vxy_reset_counter = msg_older2.vxy_reset_counter;
		msg_newer2.delta_vz = msg_older2.delta_vz;
		msg_newer2.vz_reset_counter = msg_older2.vz_reset_counter;
		msg_newer2.ax = msg_older2.ax;
		msg_newer2.ay = msg_older2.ay;
		msg_newer2.az = msg_older2.az;
		msg_newer2.heading = msg_older2.heading;
		msg_newer2.heading_var = msg_older2.heading_var;
		msg_newer2.unaided_heading = msg_older2.unaided_heading;
		msg_newer2.delta_heading = msg_older2.delta_heading;
		msg_newer2.heading_reset_counter = msg_older2.heading_reset_counter;
		msg_newer2.heading_good_for_control = msg_older2.heading_good_for_control;
		msg_newer2.tilt_var = msg_older2.tilt_var;
		msg_newer2.xy_global = msg_older2.xy_global;
		msg_newer2.z_global = msg_older2.z_global;
		msg_newer2.ref_timestamp = msg_older2.ref_timestamp;
		msg_newer2.ref_lat = msg_older2.ref_lat;
		msg_newer2.ref_lon = msg_older2.ref_lon;
		msg_newer2.ref_alt = msg_older2.ref_alt;
		msg_newer2.dist_bottom_valid = msg_older2.dist_bottom_valid;
		msg_newer2.dist_bottom_var = msg_older2.dist_bottom_var;
		msg_newer2.delta_dist_bottom = msg_older2.delta_dist_bottom;
		msg_newer2.dist_bottom_reset_counter = msg_older2.dist_bottom_reset_counter;
		msg_newer2.eph = msg_older2.eph;
		msg_newer2.epv = msg_older2.epv;
		msg_newer2.evh = msg_older2.evh;
		msg_newer2.evv = msg_older2.evv;
		msg_newer2.dead_reckoning = msg_older2.dead_reckoning;
		msg_newer2.vxy_max = msg_older2.vxy_max;
		msg_newer2.vz_max = msg_older2.vz_max;
		msg_newer2.hagl_min = msg_older2.hagl_min;
		msg_newer2.hagl_max = msg_older2.hagl_max;
	}

	static void toOlder(const MessagesNewer::Type1 &msg_newer1, const MessagesNewer::Type2 &msg_newer2,
						MessagesOlder::Type1 &msg_older1, MessagesOlder::Type2 &msg_older2) {
		// Moved from Global to Local:
		msg_older1.delta_alt = msg_newer2.delta_alt;
		msg_older1.delta_terrain = msg_newer2.delta_terrain;
		msg_older1.lat_lon_reset_counter = msg_newer2.lat_lon_reset_counter;
		msg_older1.alt_reset_counter = msg_newer2.alt_reset_counter;
		msg_older1.terrain_reset_counter = msg_newer2.terrain_reset_counter;

		// Moved from Local to Global:
		msg_older2.dist_bottom = msg_newer1.dist_bottom;

		// Global position
		msg_older1.timestamp = msg_newer1.timestamp;
		msg_older1.timestamp_sample = msg_newer1.timestamp_sample;
		msg_older1.lat = msg_newer1.lat;
		msg_older1.lon = msg_newer1.lon;
		msg_older1.alt = msg_newer1.alt;
		msg_older1.alt_ellipsoid = msg_newer1.alt_ellipsoid;
		msg_older1.eph = msg_newer1.eph;
		msg_older1.epv = msg_newer1.epv;
		msg_older1.terrain_alt = msg_newer1.terrain_alt;
		msg_older1.terrain_alt_valid = msg_newer1.terrain_alt_valid;
		msg_older1.dead_reckoning = msg_newer1.dead_reckoning;

		// Local position
		msg_older2.timestamp = msg_newer2.timestamp;
		msg_older2.timestamp_sample = msg_newer2.timestamp_sample;
		msg_older2.xy_valid = msg_newer2.xy_valid;
		msg_older2.z_valid = msg_newer2.z_valid;
		msg_older2.v_xy_valid = msg_newer2.v_xy_valid;
		msg_older2.v_z_valid = msg_newer2.v_z_valid;
		msg_older2.x = msg_newer2.x;
		msg_older2.y = msg_newer2.y;
		msg_older2.z = msg_newer2.z;
		msg_older2.delta_xy = msg_newer2.delta_vxy;
		msg_older2.xy_reset_counter = msg_newer2.xy_reset_counter;
		msg_older2.delta_z = msg_newer2.delta_z;
		msg_older2.z_reset_counter = msg_newer2.z_reset_counter;
		msg_older2.vx = msg_newer2.vx;
		msg_older2.vy = msg_newer2.vy;
		msg_older2.vz = msg_newer2.vz;
		msg_older2.z_deriv = msg_newer2.z_deriv;
		msg_older2.delta_vxy = msg_newer2.delta_vxy;
		msg_older2.vxy_reset_counter = msg_newer2.vxy_reset_counter;
		msg_older2.delta_vz = msg_newer2.delta_vz;
		msg_older2.vz_reset_counter = msg_newer2.vz_reset_counter;
		msg_older2.ax = msg_newer2.ax;
		msg_older2.ay = msg_newer2.ay;
		msg_older2.az = msg_newer2.az;
		msg_older2.heading = msg_newer2.heading;
		msg_older2.heading_var = msg_newer2.heading_var;
		msg_older2.unaided_heading = msg_newer2.unaided_heading;
		msg_older2.delta_heading = msg_newer2.delta_heading;
		msg_older2.heading_reset_counter = msg_newer2.heading_reset_counter;
		msg_older2.heading_good_for_control = msg_newer2.heading_good_for_control;
		msg_older2.tilt_var = msg_newer2.tilt_var;
		msg_older2.xy_global = msg_newer2.xy_global;
		msg_older2.z_global = msg_newer2.z_global;
		msg_older2.ref_timestamp = msg_newer2.ref_timestamp;
		msg_older2.ref_lat = msg_newer2.ref_lat;
		msg_older2.ref_lon = msg_newer2.ref_lon;
		msg_older2.ref_alt = msg_newer2.ref_alt;
		msg_older2.dist_bottom_valid = msg_newer2.dist_bottom_valid;
		msg_older2.dist_bottom_var = msg_newer2.dist_bottom_var;
		msg_older2.delta_dist_bottom = msg_newer2.delta_dist_bottom;
		msg_older2.dist_bottom_reset_counter = msg_newer2.dist_bottom_reset_counter;
		msg_older2.eph = msg_newer2.eph;
		msg_older2.epv = msg_newer2.epv;
		msg_older2.evh = msg_newer2.evh;
		msg_older2.evv = msg_newer2.evv;
		msg_older2.dead_reckoning = msg_newer2.dead_reckoning;
		msg_older2.vxy_max = msg_newer2.vxy_max;
		msg_older2.vz_max = msg_newer2.vz_max;
		msg_older2.hagl_min = msg_newer2.hagl_min;
		msg_older2.hagl_max = msg_newer2.hagl_max;
	}
};

REGISTER_TOPIC_TRANSLATION(VehicleLocalGlobalPositionV2Translation);