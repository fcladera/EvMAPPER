import numpy as np

TRIGGER_FREQ = 50  # Hz

# ------------------------------ RAW BAG CONSTANTS ------------------------------
# FLIR-related constants
FLIR_TOPIC_RAW = "/cam_sync/cam0/image_raw"
FLIR_TOPIC_INFO = "/cam_sync/cam0/camera_info"
FLIR_TOPIC_META = "/cam_sync/cam0/meta"

ALL_FLIR_TOPICS = [FLIR_TOPIC_RAW, FLIR_TOPIC_INFO, FLIR_TOPIC_META]

# VNAV-related constants
VNAV_TOPIC_COMMON = "/vectornav/raw/common"

# EC-related constants
EC_TOPIC = "/event_camera/events"

# GPS-related constants
TIM_TM2_TOPIC = "/ublox_raw/timtm2"
NAVPVT_TOPIC = "/ublox_gps_node/navpvt"

# Range-related constants
RANGE_TOPIC = "/sf000/range"

# All topics for the bag reader
ALL_INPUT_TOPICS = [
    FLIR_TOPIC_RAW,
    FLIR_TOPIC_INFO,
    FLIR_TOPIC_META,
    VNAV_TOPIC_COMMON,
    EC_TOPIC,
    TIM_TM2_TOPIC,
    NAVPVT_TOPIC,
    RANGE_TOPIC,
]

# time-sync related constants
# Golden diffs are obtained from the logic analyzer
GOLDEN_DIFFS = np.array([724.709386, 1306.995370, 1006.996216]) * 1e-3  # in s
GOLDEN_DIFFS_PPS = 3.245707914  # in s

# ------------------------------ PROCESSED BAG CONSTANTS ------------------------
OUTPUT_FLIR_IMAGE_TOPIC = "/flir/image_raw/compressed"
OUTPUT_FLIR_META_TOPIC = "/flir/meta"
OUTPUT_VNAV_IMU_TOPIC = "/VN100T/imu"
OUTPUT_VNAV_MAG_TOPIC = "/VN100T/mag"
OUTPUT_VNAV_TEMP_TOPIC = "/VN100T/temp"
OUTPUT_VNAV_PRES_TOPIC = "/VN100T/pres"
OUTPUT_EC_EVENTS_TOPIC = EC_TOPIC
OUTPUT_EC_TRIGGER_TOPIC = "/event_camera/trigger"
OUTPUT_GPS_TOPIC = "/ublox/fix"
OUTPUT_RANGE_TOPIC = RANGE_TOPIC

ALL_OUTPUT_TOPICS = [
    OUTPUT_FLIR_IMAGE_TOPIC,
    OUTPUT_FLIR_META_TOPIC,
    OUTPUT_VNAV_IMU_TOPIC,
    OUTPUT_VNAV_MAG_TOPIC,
    OUTPUT_VNAV_TEMP_TOPIC,
    OUTPUT_VNAV_PRES_TOPIC,
    OUTPUT_EC_EVENTS_TOPIC,
    OUTPUT_EC_TRIGGER_TOPIC,
    OUTPUT_GPS_TOPIC,
    OUTPUT_RANGE_TOPIC,
]

# ------------------------------ SENSOR CONSTANTS -------------------------------
FLIR_FREQ = 50  # Hz
EC_TRIGGER_FREQ = 50  # Hz
VNAV_FREQ = 400  # Hz
GPS_FREQ = 5  # Hz
RANGE_FREQ = 62.63  # Hz
FLIR_T_NS = 1 / FLIR_FREQ * 1000000000  # in ns
EC_TRIGGER_T_NS = 1 / EC_TRIGGER_FREQ * 1000000000  # in ns
VNAV_T_NS = 1 / VNAV_FREQ * 1000000000  # in ns
GPS_T_NS = 1 / GPS_FREQ * 1000000000  # in ns
RANGE_T_NS = 1 / RANGE_FREQ * 1000000000  # in ns
