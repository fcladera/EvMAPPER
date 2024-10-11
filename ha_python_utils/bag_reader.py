# Copyright 2024 Fernando Cladera <fclad@seas.upenn.edu
#
# This class is derived from Bernd's bag_reader_ros2.py
# For more information, please see https://github.com/ros-event-camera/event_camera_py
# The original copyright notice is retained below
#
# -----------------------------------------------------------------------------
# Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

# We use the bare mcap reader to get stats
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory, Decoder

# Pprint for nice printing of dictionaries
from pprint import pprint


class BagReader:
    def __init__(self, bag_name, topics):
        self.bag_path = str(bag_name)
        self.topics = topics
        # Get number of messages for each topic
        with open(self.bag_path, "rb") as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            # Get a dictionnary of the channels
            ch = reader.get_summary().channels
            topic_id = {ch[idx].topic: idx for idx in ch}
            self.all_topics = list(topic_id.keys())
            # Get channel for FLIR and number of messages
            stats = reader.get_summary().statistics.channel_message_counts
            self.duration = (
                reader.get_summary().statistics.message_end_time
                - reader.get_summary().statistics.message_start_time
            )
            self.num_msgs = {k: stats[topic_id[k]] for k in topic_id if k in topics}
        # create a sequential reader
        storage_options, converter_options = self.get_rosbag_options(self.bag_path)
        self.reader = rosbag2_py.SequentialReader()
        self.reader.open(storage_options, converter_options)
        topic_types = self.reader.get_all_topics_and_types()
        # Get type map
        self.type_map = {
            topic_types[i].name: topic_types[i] for i in range(len(topic_types))
        }
        # Add storage filter
        storage_filter = rosbag2_py.StorageFilter(topics=topics)
        self.reader.set_filter(storage_filter)
        # msg_tmp is only used when reading pairs of messages
        self.msg_tmp = None

    def has_next(self):
        """Returns True if there are more messages to read."""
        return self.reader.has_next()

    def read_all(self):
        """Read all messages in the bag file."""
        while self.reader.has_next():
            yield self.read_next()

    def read_next(self):
        """Read the next message in the bag file."""
        (topic, data, t_rec) = self.reader.read_next()
        msg_type = get_message(self.type_map[topic].type)
        msg = deserialize_message(data, msg_type)
        return (topic, msg, t_rec)

    def read_all_msg_pair(self, topic_1, topic_2):
        """Read all messages in the bag file in pairs. This is useful for fully
        synchronized with the same timestamp"""
        while self.reader.has_next():
            (topic, msg, t_rec) = self.read_next()
            if topic not in [topic_1, topic_2]:
                return
            if self.msg_tmp is not None:
                if self.msg_tmp[1].header != msg.header:
                    self.msg_tmp = (topic, msg, t_rec)
                    print("Skipping first message")
                    continue
                if topic == topic_1:
                    yield (topic, msg, t_rec), self.msg_tmp
                else:
                    yield self.msg_tmp, (topic, msg, t_rec)
                self.msg_tmp = None
            else:
                self.msg_tmp = (topic, msg, t_rec)

    def get_rosbag_options(self, path, serialization_format="cdr"):
        """Get the rosbag options for the given path and serialization
        format."""
        storage_options = rosbag2_py.StorageOptions(uri=path, storage_id="mcap")
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format=serialization_format,
            output_serialization_format=serialization_format,
        )
        return storage_options, converter_options

    def get_num_msgs(self, topic):
        """Get the number of messages for the given topic."""
        return self.num_msgs[topic]

    def get_type_map(self, topic):
        return self.type_map[topic]

    def print_stats(self, all_topics=False):
        """Print statistics for the bag file."""
        print("Bag file:\n", self.bag_path)
        if all_topics:
            print("List of all topics:")
            pprint(self.all_topics)
        else:
            print("List of topics:")
            pprint(self.topics)
        print("Number of messages per topic:")
        pprint(self.num_msgs)

    def get_duration(self):
        return self.duration
