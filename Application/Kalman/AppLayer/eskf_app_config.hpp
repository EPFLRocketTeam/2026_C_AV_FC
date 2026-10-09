#pragma once

#ifndef ESKF_ENABLED
#define ESKF_ENABLED 1
#endif

#ifndef ESKF_APP_CATCHUP_BUDGET_US
// Full-rate 6.4 kHz prediction needs a larger quantum than ingestion's
// per-loop overhead. Still bounded: service sensor FIFOs between catch-ups.
#define ESKF_APP_CATCHUP_BUDGET_US 4000
#endif

#ifndef ESKF_APP_IMU_PIPELINE_LOG_INTERVAL_US
// Static-orientation diagnostics must not exceed SD bandwidth while ARMED.
// Raw IMUs and estimator processing stay full-rate. 0 is an explicit bench
// override for unthrottled snapshots, not a supported production SD load.
#define ESKF_APP_IMU_PIPELINE_LOG_INTERVAL_US 10000
#endif

#ifndef ESKF_GPS_LEVER_ARM_X
#define ESKF_GPS_LEVER_ARM_X 0.0
#endif
#ifndef ESKF_GPS_LEVER_ARM_Y
#define ESKF_GPS_LEVER_ARM_Y 0.0
#endif
#ifndef ESKF_GPS_LEVER_ARM_Z
#define ESKF_GPS_LEVER_ARM_Z 0.0
#endif

#ifndef ESKF_APP_IMU_COUNT
#define ESKF_APP_IMU_COUNT 4
#endif

#ifndef ESKF_IMU0_POS_X
#define ESKF_IMU0_POS_X 0.0
#endif
#ifndef ESKF_IMU0_POS_Y
#define ESKF_IMU0_POS_Y 0.0
#endif
#ifndef ESKF_IMU0_POS_Z
#define ESKF_IMU0_POS_Z 0.0
#endif

#ifndef ESKF_IMU1_POS_X
#define ESKF_IMU1_POS_X 0.0
#endif
#ifndef ESKF_IMU1_POS_Y
#define ESKF_IMU1_POS_Y 0.0
#endif
#ifndef ESKF_IMU1_POS_Z
#define ESKF_IMU1_POS_Z 0.0
#endif

#ifndef ESKF_APP_BARO_COUNT
#define ESKF_APP_BARO_COUNT 4
#endif

#ifndef ESKF_LAUNCH_LATITUDE_DEG
#define ESKF_LAUNCH_LATITUDE_DEG 46.848199
#endif
