import sys
import rosbag
import time
import subprocess
import yaml
import rospy
import os
import argparse
import math
from shutil import move


def status(length, percent):
    sys.stdout.write('\x1B[2K')  # Erase entire current line
    sys.stdout.write('\x1B[0E')  # Move to the beginning of the current line
    progress = "Progress: ["
    for i in range(0, length):
        if i < length * percent:
            progress += '='
        else:
            progress += ' '
    progress += "] " + str(round(percent * 100.0, 2)) + "%"
    sys.stdout.write(progress)
    sys.stdout.flush()


def main(args):
    parser = argparse.ArgumentParser(
        description='Reorder a bagfile based on header timestamps.')
    parser.add_argument('bagfile', nargs=1, help='input bag file')
    parser.add_argument('--max-offset', nargs=1,
                        help='max time offset (sec) to correct.', default='60000000000', type=float)
    args = parser.parse_args()

    # Get bag duration

    bagfile = args.bagfile[0]

    info_dict = yaml.load(subprocess.Popen(
        ['rosbag', 'info', '-y', bagfile], stdout=subprocess.PIPE).communicate()[0])
    duration = info_dict['duration']
    start_time = info_dict['start']

    orig = os.path.splitext(bagfile)[0] + ".orig.bag"

    move(bagfile, orig)
    last_msg_stamp = None
    duration = 250

    min_stamp = None
    max_stamp = None
    with rosbag.Bag(bagfile, 'w') as outbag:

        last_time = time.process_time()
        for topic, msg, t in rosbag.Bag(orig).read_messages():
            """if msg.header.stamp.to_sec() < 1600171135.3 or msg.header.stamp.to_sec() > 1600171328.1:
                continue
            if last_msg_stamp is None:
                last_msg_stamp = msg.header.stamp.to_sec()
                min_stamp = last_msg_stamp
                max_stamp = last_msg_stamp

            if msg.header.stamp.to_sec() < min_stamp:
                min_stamp = msg.header.stamp.to_sec()
            if msg.header.stamp.to_sec() > max_stamp:
                max_stamp = msg.header.stamp.to_sec()"""
            if time.process_time() - last_time > .1:
                percent = (t.to_sec() - start_time) / duration
                status(40, percent)
                last_time = time.process_time()

            # This also replaces tf timestamps under the assumption
            # that all transforms in the message share the same timestamp
            if topic == "/tf" and msg.transforms:
                # Writing transforms to bag file 1 second ahead of time to ensure availability
                diff = math.fabs(
                    msg.transforms[0].header.stamp.to_sec() - t.to_sec())
                outbag.write(topic, msg, msg.transforms[0].header.stamp)
            elif msg._has_header:
                diff = math.fabs(msg.header.stamp.to_sec() - t.to_sec())
                outbag.write(topic, msg, msg.header.stamp)
            else:
                outbag.write(topic, msg, t)
    status(40, 1)
    print("\ndone")
    print('min stamp: ' + str(min_stamp))
    print('max stamp: ' + str(max_stamp))


if __name__ == "__main__":
    main(sys.argv[1:])
