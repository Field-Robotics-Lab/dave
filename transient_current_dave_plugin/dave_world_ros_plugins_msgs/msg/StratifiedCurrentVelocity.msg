# Publishes depths and velocities read from the csv database

# Depths
float32[] depths

# Velocities
geometry_msgs/Vector3[] velocities

# Tide time (GMT)
string[] timeGMT

# Tide velocities
float32[] tideVelocities

# Tide direction
float32 ebbDirection
float32 floodDirection

# World start time (GMT)
string worldStartTime
