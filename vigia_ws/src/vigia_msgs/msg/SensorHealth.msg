# Sensor bridge health counters — published at 1 Hz on /vigia/sensor_health
std_msgs/Header header
uint64  imu_count
uint64  gps_count
uint64  ping_count
uint32  imu_seq_gaps
uint32  gps_seq_gaps
uint32  parse_errors
float32 imu_hz_measured
bool    port_open
