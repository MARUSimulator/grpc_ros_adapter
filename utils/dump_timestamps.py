#!/usr/bin/env python
import sys
import roslib
import rospy
import rosbag
from rospy import rostime
import argparse
import os
import shutil
import csv


def parse_args():
    parser = argparse.ArgumentParser(
        prog='bagmerge.py',
        description='Merges two bagfiles.')
    parser.add_argument('-o', type=str, help='name of the output file',
                        default=None, metavar="output_file")
    parser.add_argument('-t', type=str, help='topics which should be merged to the main bag',
                        default=None, metavar="topics")
    parser.add_argument('-i', help='reindex bagfile',
                        default=False, action="store_true")
    parser.add_argument('main_bagfile', type=str,
                        help='path to a bagfile, which will be the main bagfile')
    args = parser.parse_args()
    return args


def dump_stamps(main_bagfile):
    # get min and max time in bagfile

    # check output file

    # merge bagfile
    filename = main_bagfile + '_stamps.csv'
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',',
                            quotechar='|', quoting=csv.QUOTE_MINIMAL)
        writer.writerow(['topic', 'bag_time', 'header_time'])
        for topic, msg, t in rosbag.Bag(main_bagfile).read_messages():
            if topic != '/velodyne_points':
                continue
            try:
                writer.writerow([topic, t.to_sec(), msg.header.stamp.to_sec()])
            except:
                writer.writerow([topic, t.to_sec(), 'NA'])


if __name__ == "__main__":
    args = parse_args()
    if args.t != None:
        args.t = args.t.split(',')
    dump_stamps(args.main_bagfile,
                )
