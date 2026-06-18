#pragma once
/**
 * Shared wire contract — must match firmware/src/atecc608a_driver.h byte-for-byte.
 */
#ifndef VIGIA_PACKET_H
#define VIGIA_PACKET_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(push, 1)
typedef struct {
    uint8_t  device_id[16];
    uint64_t timestamp_us;
    uint32_t sequence;
    float    qw, qx, qy, qz;
    float    ax, ay, az;
    uint8_t  cal_status;
    uint8_t  _pad0[3];
    double   latitude;
    double   longitude;
    float    speed_ms;
    uint8_t  fix_type;
    uint8_t  satellites;
    uint8_t  _pad1[2];
    uint8_t  _pad2[12];
} EtHashInput;
#pragma pack(pop)

#define ET_HASH_INPUT_SIZE 96

#pragma pack(push, 1)
typedef struct {
    uint8_t  magic;
    uint8_t  version;
    uint64_t timestamp_us;
    uint32_t sequence;
    float    qw, qx, qy, qz;
    float    ax, ay, az;
    uint8_t  cal_status;
    uint8_t  _imu_pad[3];
    double   latitude;
    double   longitude;
    float    speed_ms;
    uint8_t  fix_type;
    uint8_t  satellites;
    uint8_t  _gps_pad[1];
    uint8_t  et_hash[32];
    uint8_t  ecdsa_sig[64];
    uint8_t  wire_pad[8];
} SignedEtPacket;
#pragma pack(pop)

#define SIGNED_ET_PACKET_SIZE 173
#define SIGNED_ET_MAGIC       0xE7u
#define SIGNED_ET_VERSION     0x02u

#ifdef __cplusplus
}
#endif

#endif /* VIGIA_PACKET_H */
