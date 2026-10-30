#ifndef DRONEFC_UDP_TELEMETRY_H
#define DRONEFC_UDP_TELEMETRY_H

#include <stdint.h>
#include <stddef.h>

#define UDP_TELEMETRY_MAGIC UINT32_C(0x314C5444)
#define UDP_TELEMETRY_VERSION 4

// Wire format: little-endian integers and IEEE-754 binary32 floats.
typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t packet_bytes;
  uint32_t sequence;
  uint32_t control_period_us;
  uint64_t sample_time_us;
  float acc_g[3];
  float gyro_dps[3];
  float attitude_deg[2];
  float attitude_sp_deg[2];
  float rate_sp_dps[2];
  float baro_alt_cm;
  float altitude_cm;
  float climb_cms;
  float climb_sp_cms;
  uint16_t rc[7];
  uint16_t reserved_rc;
  uint16_t esc[4];
  uint32_t baro_prev_work_us;
  uint32_t baro_period_us;
  uint32_t udp_prev_work_us;
  uint32_t baro_work_overruns;
  uint32_t udp_send_errors;
  float baro_pressure_pa;
  float baro_temperature_c;
  uint32_t baro_sample_id;
  uint32_t reserved;
  uint8_t flight_mode;
  uint8_t armed;
  uint8_t rc_ok;
  uint8_t baro_present;
  uint64_t baro_sample_time_us;
  float acc_earth_z_g;
  float acc_bias_cms2;
  float acc_corrected_cms2;
  uint32_t baro_age_us;
  uint32_t udp_snapshot_skips;
  uint8_t baro_healthy;
  uint8_t baro_correction_used;
  uint8_t baro_reference_valid;
  uint8_t reserved_v3;
} UdpTelemetryPacket;

#ifdef __cplusplus
static_assert(sizeof(float) == 4, "Telemetry requires binary32 floats");
static_assert(sizeof(UdpTelemetryPacket) == 184, "Telemetry wire layout changed");
static_assert(offsetof(UdpTelemetryPacket, sample_time_us) == 16, "Control timestamp offset changed");
static_assert(offsetof(UdpTelemetryPacket, rc) == 88, "RC offset changed");
static_assert(offsetof(UdpTelemetryPacket, esc) == 104, "ESC offset changed");
static_assert(offsetof(UdpTelemetryPacket, baro_pressure_pa) == 132, "Pressure offset changed");
static_assert(offsetof(UdpTelemetryPacket, baro_sample_time_us) == 152, "Barometer timestamp offset changed");
static_assert(offsetof(UdpTelemetryPacket, baro_healthy) == 180, "Status offset changed");
#else
_Static_assert(sizeof(float) == 4, "Telemetry requires binary32 floats");
_Static_assert(sizeof(UdpTelemetryPacket) == 184, "Telemetry wire layout changed");
_Static_assert(offsetof(UdpTelemetryPacket, rc) == 88, "RC offset changed");
_Static_assert(offsetof(UdpTelemetryPacket, esc) == 104, "ESC offset changed");
_Static_assert(offsetof(UdpTelemetryPacket, baro_pressure_pa) == 132, "Pressure offset changed");
_Static_assert(offsetof(UdpTelemetryPacket, baro_sample_time_us) == 152, "Barometer timestamp offset changed");
_Static_assert(offsetof(UdpTelemetryPacket, baro_healthy) == 180, "Status offset changed");
#endif

#endif
