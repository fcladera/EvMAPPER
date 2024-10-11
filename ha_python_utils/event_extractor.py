#!/usr/bin/env python3
import sys

sys.path.append("..")
import ha_python_utils.constants as const
from ha_python_utils.bag_reader import BagReader
from rclpy.serialization import serialize_message
import numpy as np
from pprint import pprint
import matplotlib.pyplot as plt
from pathlib import Path
import rosbag2_py
from event_camera_py import Decoder
import copy
import time
import collections
import cv2
import shutil
import pdb
import utm
import zipfile
from event_camera_py import Decoder


class EventExtractor:
    def __init__(self, bag_name):
        # Check that this is a synchronized bag
        assert "synchronized_0.mcap" == bag_name.name

        self.bag_reader = BagReader(bag_name, [const.OUTPUT_EC_EVENTS_TOPIC])
        self.decoder = Decoder()

        # Get output folder from input bag
        self.txt_file = bag_name.parents[1] / "extracted_events" / "events.txt"

        # Create dir e2vid if it does not exist
        self.txt_file.parents[0].mkdir(parents=True, exist_ok=True)

        # Delete txt file if it exists
        if self.txt_file.exists():
            self.txt_file.unlink()

    def extract(self):
        print(f"Extracting events to {self.txt_file}")
        with open(self.txt_file, "w") as f:
            # Write the header of the file, the resolution of the camera
            f.write("1280 720\n")
            for i, (topic, msg, timestamp) in enumerate(self.bag_reader.read_all()):
                self.decoder.decode(msg)  # Always decode
                events = self.decoder.get_cd_events()
                for x, y, p, ts in events:
                    f.write(f"{ts/1e6:.9f} {x} {y} {p}\n")
                if i % 1000 == 0:
                    print(timestamp / 1e9)


if __name__ == "__main__":
    import argparse

    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("--input_bag", type=str, required=True)
    args = arg_parser.parse_args()

    # Check that input bag is a file and exists
    input_bag = Path(args.input_bag)
    assert input_bag.exists(), "Input bag does not exist"
    assert input_bag.is_file(), "Input bag is not a file"

    event_extractor = EventExtractor(input_bag)
    event_extractor.extract()
