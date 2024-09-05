#!/usr/bin/env python3

import cv2
import matplotlib.pyplot as plt
import numpy as np
import pdb
import rosbag2_py
import sensor_msgs.msg
import shutil
import std_msgs.msg
import sys
import yaml
from event_camera_py import Decoder
from pathlib import Path
from rclpy.serialization import serialize_message

sys.path.append("..")
import ha_python_utils.constants as const
from ha_python_utils.bag_reader import BagReader


class BagSynchronizer:
    def __init__(self, bag_name):
        # bag_name should be a pathlib.Path object
        assert isinstance(bag_name, Path), "bag_name should be a pathlib.Path object"
        self.bag_name = bag_name
        sync_yaml = bag_name.parents[1] / "sync_info.yaml"
        self.synced_bag = bag_name.parents[1] / "synchronized"
        # Read the yaml
        with open(sync_yaml, "r") as f:
            self.time_sync_dict = yaml.load(f, yaml.SafeLoader)
        # The event camera will provide the reference timing
        time_offset_ec_post_cut = self.time_sync_dict["offsets"]["ec"]["time_offset_post_cut"]  # us
        time_offset_ec_post_cut_ns = time_offset_ec_post_cut * 1000
        self.reference_time_ns = time_offset_ec_post_cut_ns
        self.delete_synced_bag()

    def delete_synced_bag(self):
        """Delete the synced bag if it exists"""
        if (
            self.synced_bag.exists()
            and self.synced_bag.is_dir()
            and any(self.synced_bag.iterdir())
        ):
            shutil.rmtree(self.synced_bag)

    def synchronize(self):
        # Create a writer object
        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(uri=str(self.synced_bag), storage_id="mcap"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr", output_serialization_format="cdr"
            ),
        )
        self.write_flir_messages(writer)
        self.write_ec_messages(writer)
        self.write_range_messages(writer)
        self.write_vnav_messages(writer)
        self.write_ublox_messages(writer)
        del writer

        print(f"Bag synchronized to {self.synced_bag}")

    def write_flir_messages(self, writer):
        # Write FLIR messages
        print("Writing FLIR messages")
        bag_reader = BagReader(
            self.bag_name, [const.FLIR_TOPIC_RAW, const.FLIR_TOPIC_META]
        )
        time_offset_flir = self.time_sync_dict["offsets"]["flir"]["time_offset"]
        first_sample_flir = self.time_sync_dict["offsets"]["flir"]["first_sample"]
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=const.OUTPUT_FLIR_IMAGE_TOPIC,
                type="sensor_msgs/msg/CompressedImage",
                serialization_format="cdr",
            )
        )
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=const.OUTPUT_FLIR_META_TOPIC,
                type="flir_camera_msgs/msg/ImageMetaData",
                serialization_format="cdr",
            )
        )
        count_flir = 0
        for (raw_topic, raw_msg, _), (
            meta_topic,
            meta_msg,
            _,
        ) in bag_reader.read_all_msg_pair(const.FLIR_TOPIC_RAW, const.FLIR_TOPIC_META):
            assert raw_topic == const.FLIR_TOPIC_RAW
            assert meta_topic == const.FLIR_TOPIC_META
            # Skip messages until the offset is found. This message is ts = 0
            meta_ts = meta_msg.camera_time - meta_msg.exposure_time * 1000
            if meta_ts < time_offset_flir:
                count_flir += 1
                continue
            elif meta_ts == time_offset_flir:
                if count_flir != first_sample_flir:
                    print("COUNT IS NOT THE SAME, CHECK FOR SKIPPED SAMPLE")
                assert np.abs(count_flir - first_sample_flir) <= 1
            ts_corrected = meta_ts - time_offset_flir + self.reference_time_ns
            compressed_msg = sensor_msgs.msg.CompressedImage()
            compressed_msg.header.frame_id = "FLIR"
            compressed_msg.header.stamp.sec = ts_corrected // 1000000000
            compressed_msg.header.stamp.nanosec = ts_corrected % 1000000000
            compressed_msg.format = "png"
            flir_width = 2048
            flir_height = 1536
            arr = np.frombuffer(raw_msg.data, np.uint8).reshape(flir_height, flir_width)
            # img =  cv2.cvtColor(arr, cv2.COLOR_BayerRG2RGB)
            # Compress images bayered for better compression rate
            _, encoded_image = cv2.imencode(
                ".png", arr, [cv2.IMWRITE_PNG_COMPRESSION, 9]
            )
            compressed_msg.data = encoded_image.tobytes()
            writer.write(
                const.OUTPUT_FLIR_IMAGE_TOPIC,
                serialize_message(compressed_msg),
                ts_corrected,
            )
            meta_msg.header = compressed_msg.header
            writer.write(
                const.OUTPUT_FLIR_META_TOPIC, serialize_message(meta_msg), ts_corrected
            )

    def write_range_messages(self, writer):
        print("Writing Range messages")
        bag_reader = BagReader(self.bag_name, [const.RANGE_TOPIC])
        time_offset_range = self.time_sync_dict["offsets"]["sf000"]["time_offset"]
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=const.OUTPUT_RANGE_TOPIC,
                type="sensor_msgs/msg/Range",
                serialization_format="cdr",
            )
        )
        ts_range = np.zeros(bag_reader.get_num_msgs(const.RANGE_TOPIC), dtype=np.int64)
        i = 0
        for topic, msg, timestamp in bag_reader.read_all():
            ts_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
            if ts_ns < time_offset_range:
                continue
            ts_ns_corrected = ts_ns - time_offset_range + self.reference_time_ns
            ts_range[i] = ts_ns_corrected
            i += 1
            range_msg = sensor_msgs.msg.Range()
            range_msg.header.frame_id = "SF000"
            range_msg.header.stamp.sec = ts_ns_corrected // 1000000000
            range_msg.header.stamp.nanosec = ts_ns_corrected % 1000000000
            range_msg.radiation_type = 1
            range_msg.min_range = 0.2
            range_msg.max_range = 50.0
            range_msg.range = msg.first_return_filtered / 1000.0
            writer.write(
                const.OUTPUT_RANGE_TOPIC, serialize_message(range_msg), ts_ns_corrected
            )

    def write_vnav_messages(self, writer):
        # Write VNAV messages
        print("Writing VNAV messages")
        imu_topics = {
            const.OUTPUT_VNAV_IMU_TOPIC: "sensor_msgs/msg/Imu",
            const.OUTPUT_VNAV_MAG_TOPIC: "sensor_msgs/msg/MagneticField",
            const.OUTPUT_VNAV_TEMP_TOPIC: "sensor_msgs/msg/Temperature",
            const.OUTPUT_VNAV_PRES_TOPIC: "sensor_msgs/msg/FluidPressure",
        }
        for imu_topic in imu_topics:
            writer.create_topic(
                rosbag2_py.TopicMetadata(
                    name=imu_topic,
                    type=imu_topics[imu_topic],
                    serialization_format="cdr",
                )
            )
        bag_reader = BagReader(self.bag_name, [const.VNAV_TOPIC_COMMON])
        time_offset_vnav = self.time_sync_dict["offsets"]["vnav"]["time_offset"]
        first_sample_vnav = self.time_sync_dict["offsets"]["vnav"]["first_sample"]
        count_vnav = 0
        for topic, msg, timestamp in bag_reader.read_all():
            # Skip messages until the offset is found.
            if count_vnav < first_sample_vnav:
                count_vnav += 1
                continue
            ts_corrected = msg.timestartup - time_offset_vnav + self.reference_time_ns
            header = std_msgs.msg.Header()
            header.frame_id = "VN100T"
            header.stamp.sec = ts_corrected // 1000000000
            header.stamp.nanosec = ts_corrected % 1000000000
            # Fill in the messages
            imu_msg = sensor_msgs.msg.Imu()
            imu_msg.linear_acceleration = msg.accel  # These are the compensated ones
            imu_msg.angular_velocity = msg.angularrate
            imu_msg.orientation = msg.quaternion  # ENU, may need to convert to NED
            mag_msg = sensor_msgs.msg.MagneticField()
            mag_msg.magnetic_field = msg.magpres_mag
            temp_msg = sensor_msgs.msg.Temperature()
            temp_msg.temperature = msg.magpres_temp
            press_msg = sensor_msgs.msg.FluidPressure()
            press_msg.fluid_pressure = msg.magpres_pres
            press_msg.header = temp_msg.header = mag_msg.header = imu_msg.header = (
                header
            )
            writer.write(
                const.OUTPUT_VNAV_IMU_TOPIC, serialize_message(imu_msg), ts_corrected
            )
            writer.write(
                const.OUTPUT_VNAV_MAG_TOPIC, serialize_message(mag_msg), ts_corrected
            )
            writer.write(
                const.OUTPUT_VNAV_TEMP_TOPIC, serialize_message(temp_msg), ts_corrected
            )
            writer.write(
                const.OUTPUT_VNAV_PRES_TOPIC, serialize_message(press_msg), ts_corrected
            )

    def write_ec_messages(self, writer):
        print("Writing EC messages")
        bag_reader = BagReader(self.bag_name, [const.EC_TOPIC])
        writer.create_topic(bag_reader.get_type_map(const.EC_TOPIC))
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=const.OUTPUT_EC_TRIGGER_TOPIC,
                type="std_msgs/msg/Header",
                serialization_format="cdr",
            )
        )
        first_sample_ec = self.time_sync_dict["offsets"]["ec"]["first_sample"]
        decoder = Decoder()
        count_ec = 0
        sequence_started = False
        time_offset_ec_pre_cut = self.time_sync_dict["offsets"]["ec"]["time_offset_pre_cut"]  # us
        time_offset_ec_post_cut = self.time_sync_dict["offsets"]["ec"]["time_offset_post_cut"]  # us
        for topic, encoded_msg, timestamp in bag_reader.read_all():
            # Wait until we get the first trigger
            if not sequence_started:
                decoder.decode(encoded_msg)
                trig_events = decoder.get_ext_trig_events()
                if len(trig_events) > 0 and trig_events[0][0] == 1:
                    trigger_ts = trig_events[0][1]
                    if trigger_ts < time_offset_ec_pre_cut:
                        count_ec += 1
                        continue
                    else:
                        sequence_started = True
                        assert first_sample_ec == count_ec
                        # Delete decoder, will instantiate later
                        decoder = None
                elif len(trig_events) > 2:
                    sys.exit("Too many trigger events")

            # Just copy the packets afterwards.
            if sequence_started:
                if decoder is None:
                    decoder = Decoder()
                    decoder.decode(encoded_msg)
                    trig_events = decoder.get_ext_trig_events()
                    assert trig_events[0][1] == time_offset_ec_post_cut
                else:
                    decoder.decode(encoded_msg)
                    trig_events = decoder.get_ext_trig_events()
                if len(trig_events) > 0 and trig_events[0][0] == 1:
                    last_ts_timestamp = trig_events[0][1]
                    trig_header = std_msgs.msg.Header()
                    trig_header.stamp.sec = int(last_ts_timestamp) // 1000000
                    trig_header.stamp.nanosec = int(last_ts_timestamp) % 1000000 * 1000
                    trig_header.frame_id = "SilkyHD"
                    last_trig_header = trig_header
                    writer.write(
                        const.OUTPUT_EC_TRIGGER_TOPIC,
                        serialize_message(trig_header),
                        last_ts_timestamp * 1000,
                    )
                encoded_msg.header = last_trig_header
                writer.write(
                    const.OUTPUT_EC_EVENTS_TOPIC,
                    serialize_message(encoded_msg),
                    last_ts_timestamp * 1000,
                )

    def write_ublox_messages(self, writer):
        print("Writing UBLOX messages")
        bag_reader = BagReader(self.bag_name, [const.TIM_TM2_TOPIC, const.NAVPVT_TOPIC])
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=const.OUTPUT_GPS_TOPIC,
                type="sensor_msgs/msg/NavSatFix",
                serialization_format="cdr",
            )
        )
        time_offset_ublox = self.time_sync_dict["offsets"]["ublox"]["time_offset"]
        first_sample_ublox = self.time_sync_dict["offsets"]["ublox"]["first_sample"]
        i = 0
        sequence_started = False
        for topic, msg, timestamp in bag_reader.read_all():
            if topic == const.TIM_TM2_TOPIC and not sequence_started:
                if msg.rising_edge_count == 0:
                    continue
                tim_tm2_msg_tow = (
                    msg.tow_ms_r * 1000000 + msg.tow_sub_ms_r + 18000000000
                )

                if tim_tm2_msg_tow == time_offset_ublox:
                    assert i == first_sample_ublox
                    sequence_started = True
                i += 1
            if sequence_started and topic == const.NAVPVT_TOPIC:
                pvt_tow = msg.i_tow * 1000000
                # Do not use msg.nano as nano is also counting the milliseconds
                # sometimes we can get a NAVPVT message before the TIM_TM2 message
                if pvt_tow < time_offset_ublox:
                    continue
                ts_corrected = pvt_tow - time_offset_ublox + self.reference_time_ns
                gps_msg = sensor_msgs.msg.NavSatFix()
                gps_msg.header.frame_id = "ZED_F9P"
                gps_msg.header.stamp.sec = ts_corrected // 1000000000
                gps_msg.header.stamp.nanosec = ts_corrected % 1000000000
                gps_msg.latitude = msg.lat * 1e-7
                gps_msg.longitude = msg.lon * 1e-7
                gps_msg.altitude = msg.height / 1e3
                writer.write(
                    const.OUTPUT_GPS_TOPIC, serialize_message(gps_msg), ts_corrected
                )


if __name__ == "__main__":
    import argparse

    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("--input_bag", type=str, required=True)
    args = arg_parser.parse_args()

    # Check that input bag is a file and exists
    input_bag = Path(args.input_bag)
    assert input_bag.exists(), "Input bag does not exist"
    assert input_bag.is_file(), "Input bag is not a file"
    bag_sync = BagSynchronizer(input_bag)
    bag_sync.synchronize()
