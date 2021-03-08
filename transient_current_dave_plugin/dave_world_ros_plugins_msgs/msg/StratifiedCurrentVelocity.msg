# Publishes depths and velocities read from the csv database

# Depths
float32[] depths

# Velocities
geometry_msgs/Vector3[] velocities

# Tide time (GMT)
string[] timeGMT

# Tide velocities
float32[] tideVelocities

# Tide constituents
bool tideConstituents
float32 M2amp
float32 M2phase
float32 M2speed
float32 S2amp
float32 S2phase
float32 S2speed
float32 N2amp
float32 N2phase
float32 N2speed

# Tide direction
float32 ebbDirection
float32 floodDirection

# World start time (GMT)
string worldStartTime
