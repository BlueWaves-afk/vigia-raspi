# DePIN signed event telemetry packet (Phase 2 wire protocol)
std_msgs/Header header
uint64 timestamp_us
uint32 sequence

# IMU
float32 q_w
float32 q_x
float32 q_y
float32 q_z
float32 lin_accel_x
float32 lin_accel_y
float32 lin_accel_z
uint8   calibration_status

# GPS
float64 lat
float64 lon
float32 speed_ms
uint8   fix_type
uint8   satellites_used
float32 hdop
bool    valid_fix

# Signing — et_hash is SHA-256 of EtHashInput (96 bytes) via ATECC608A
# ecdsa_sig is secp256r1 ECDSA-SHA256 via ATECC608A slot 0 private key
# sig_valid is false until SE is wired and mbedTLS verify passes
uint8[32] et_hash
uint8[64] ecdsa_sig
bool      sig_valid
