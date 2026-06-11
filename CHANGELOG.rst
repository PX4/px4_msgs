Changelog for package px4_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The version numbers of this package track the corresponding `PX4 Autopilot
<https://github.com/PX4/PX4-Autopilot>`_ release line. The message and service
definitions are generated and synchronized automatically from the uORB
definitions in PX4-Autopilot, so the entries below are produced from the tagged
release history rather than by manual edits (see
``scripts/generate_changelog.sh``).

1.17.0 (2026-03-22)
-------------------
* Synchronized with PX4 Autopilot 1.17.0.
* Added: BatteryInfo, DronecanNodeStatus, FixedWingLateralGuidanceStatus, FixedWingLateralSetpoint, FixedWingLateralStatus, FixedWingLongitudinalSetpoint, FixedWingRunwayControl, LateralControlConfiguration, LongitudinalControlConfiguration, NeuralControl, RoverSpeedSetpoint, RoverSpeedStatus, SensorGnssStatus.
* Removed: AckermannVelocitySetpoint, DifferentialVelocitySetpoint, NpfgStatus, RoverVelocityStatus.
* Changed 44 message/service definition(s).
* Contributors: Beniamino Pozzan

1.16.2 (2025-08-07)
-------------------
* PX4 point release; message and service definitions identical to 1.16.1.
* Contributors: PX4 BuildBot

1.16.1 (2025-08-07)
-------------------
* PX4 point release; message and service definitions identical to 1.16.0.
* Contributors: PX4 BuildBot

1.16.0 (2025-08-07)
-------------------
* Synchronized with PX4 Autopilot 1.16.0.
* Added: AckermannVelocitySetpoint, DifferentialVelocitySetpoint, DistanceSensorModeChangeRequest, FuelTankStatus, InternalCombustionEngineControl, NavigatorStatus, OpenDroneIdArmStatus, OpenDroneIdOperatorId, OpenDroneIdSelfId, OpenDroneIdSystem, PurePursuitStatus, RoverAttitudeSetpoint, RoverAttitudeStatus, RoverPositionSetpoint, RoverRateSetpoint, RoverRateStatus, RoverSteeringSetpoint, RoverThrottleSetpoint, RoverVelocityStatus, TrajectorySetpoint6dof.
* Removed: Buffer128, CollisionReport, DifferentialDriveSetpoint, TrajectoryBezier, TrajectoryWaypoint, VehicleTrajectoryBezier, VehicleTrajectoryWaypoint.
* Changed 71 message/service definition(s).
* Contributors: Beniamino Pozzan, Claudio Chies, GuillaumeLaine, Mathieu David, PX4 BuildBot

1.15.4 (2025-03-20)
-------------------
* PX4 point release; message and service definitions identical to 1.15.3.
* Contributors: PX4 BuildBot

1.15.3 (2025-03-20)
-------------------
* Synchronized with PX4 Autopilot 1.15.3.
* Changed 3 message/service definition(s).
* Contributors: Beniamino Pozzan

1.15.2 (2024-10-06)
-------------------
* PX4 point release; message and service definitions identical to 1.15.1.
* Contributors: PX4 BuildBot

1.15.1 (2024-10-06)
-------------------
* Synchronized with PX4 Autopilot 1.15.1.
* Changed 1 message/service definition(s).
* Contributors: Beniamino Pozzan, PX4 BuildBot

1.15.0 (2024-06-17)
-------------------
* Synchronized with PX4 Autopilot 1.15.0.
* Added: ArmingCheckReply, ArmingCheckRequest, Buffer128, CanInterfaceStatus, ConfigOverrides, DatamanRequest, DatamanResponse, DifferentialDriveSetpoint, FigureEightStatus, FlightPhaseEstimation, GeofenceStatus, GotoSetpoint, GpioConfig, GpioIn, GpioOut, GpioRequest, MessageFormatRequest, MessageFormatResponse, ParameterResetRequest, ParameterSetUsedRequest, ParameterSetValueRequest, ParameterSetValueResponse, RegisterExtComponentReply, RegisterExtComponentRequest, RtlStatus, SensorAirflow, UnregisterExtComponent, VehicleCommand, VelocityLimits, WheelEncoders.
* Changed 35 message/service definition(s).
* Contributors: Audrow Nash, Beniamino Pozzan, dependabot[bot], Federico Ciresola, PX4 BuildBot

1.14.0 (2023-11-11)
-------------------
* Synchronized with PX4 Autopilot 1.14.0.
* Added: EstimatorAidSource1d, EstimatorAidSource2d, EstimatorAidSource3d, EstimatorBias3d, FailsafeFlags, FollowTargetEstimator, FollowTargetStatus, Gripper, HealthReport, LandingGearWheel, LaunchDetectionStatus, ModeCompleted, NormalizedUnsignedSetpoint, QshellReq, SensorOpticalFlow, SensorUwb, TiltrotorExtraControls, VehicleOpticalFlow, VehicleOpticalFlowVel.
* Removed: ActuatorControls, ActuatorControls0, ActuatorControls1, ActuatorControls2, ActuatorControls3, ActuatorControlsStatus1, ActuatorControlsVirtualFw, ActuatorControlsVirtualMc, ActuatorOutputsSim, CommanderState, EstimatorAttitude, EstimatorGlobalPosition, EstimatorInnovationTestRatios, EstimatorInnovationVariances, EstimatorLocalPosition, EstimatorOdometry, EstimatorOpticalFlowVel, EstimatorVisualOdometryAligned, EstimatorWind, FwVirtualAttitudeSetpoint, GimbalV1Command, ManualControlInput, McVirtualAttitudeSetpoint, ObstacleDistanceFused, OpticalFlow, OrbTestMediumMulti, OrbTestMediumQueue, OrbTestMediumQueuePoll, OrbTestMediumWrapAround, Safety, SafetyButton, SensorsStatusBaro, SensorsStatusMag, TestMotor, Timesync, UwbDistance, UwbGrid, VehicleAngularAcceleration, VehicleAngularVelocityGroundtruth, VehicleAttitudeGroundtruth, VehicleGlobalPositionGroundtruth, VehicleGpsPosition, VehicleLocalPositionGroundtruth, VehicleMocapOdometry, VehicleStatusFlags, VehicleTrajectoryWaypointDesired, VehicleVisionAttitude, VehicleVisualOdometry, WheelEncoders.
* Changed 72 message/service definition(s).
* Contributors: Beniamino Pozzan, Daniel Agar, PX4 BuildBot

1.13.0 (2023-07-10)
-------------------
* Initial tracked release, synchronized with PX4 Autopilot 1.13.0.
* Contributors: PX4 BuildBot
