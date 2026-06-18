std_msgs/Header             header
string                      device_id
float32                     rri_score
float32                     iss_score
float32                     yolo_confidence
float32                     geometry_confidence
float32                     temporal_confidence
vigia_msgs/DetectionArray   detections
vigia_msgs/DepthMap         depth_map
vigia_msgs/ImuSample        imu_sample
vigia_msgs/GpsPvt           gps_pvt
vigia_msgs/SignedEt         signed_et
float32[]                   spatial_latent

# Degraded-mode reporting (design spec §6.4). When the Pico is in stub mode
# (no IMU/GPS) the RRI is computed from a renormalized subset of terms.
bool                        degraded        # true if not full-fusion (no IMU/ISS)
uint8                       rri_tier        # 0=none 1=vision 2=vision+depth 3=full
