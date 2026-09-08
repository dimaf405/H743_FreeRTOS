
# EKF2 回放按源速率排空；四槽覆盖 Logger 5 ms 兜底扫描期间的磁力计突发，
# 同时避免为低频源长期占用更多队列 RAM。
uint8 ORB_QUEUE_LENGTH = 4

uint64 timestamp            # time since system start (microseconds)

uint64 timestamp_sample     # the timestamp of the raw data (microseconds)

uint32 device_id            # unique device ID for the selected magnetometer

float32[3] magnetometer_ga  # Magnetic field in the FRD body frame XYZ-axis in Gauss

uint8 calibration_count     # Calibration changed counter. Monotonically increases whenever calibration changes.
