# Publishes depths and velocities read from the csv database

# Depths
float32[] depths

# Velocities
geometry_msgs/Vector3[] velocities

# Tide time (GMT)
int16[] timeGMTYear
int16[] timeGMTMonth
int16[] timeGMTDay
int16[] timeGMTHour
int16[] timeGMTMinute

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
int16 worldStartTimeYear
int16 worldStartTimeMonth
int16 worldStartTimeDay
int16 worldStartTimeHour
int16 worldStartTimeMinute