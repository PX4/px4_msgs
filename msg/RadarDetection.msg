# Radar detection report
uint64 timestamp		# time since system start (microseconds)

uint8 track_id # Unique track ID - 1 for target, 2 for interceptor

# position
float32 x # x position in meters - North
float32 y # y position in meters - East
float32 z # z position in meters - Down

# velocity
float32 vx # velocity in meters/sec - North
float32 vy # velocity in meters/sec - East
float32 vz # velocity in meters/sec - Down

# position covariance
float32 pos_cov_x # x position covariance in meters
float32 pos_cov_y # y position covariance in meters
float32 pos_cov_z # z position covariance in meters

# velocity covariance
float32 vel_cov_x # x velocity covariance in meters/sec
float32 vel_cov_y # y velocity covariance in meters/sec
float32 vel_cov_z # z velocity covariance in meters/sec
