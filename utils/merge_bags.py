#!/usr/bin/env python
import sys
import roslib
import rospy
import rosbag
from rospy import rostime
import argparse
import os
import shutil


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
    parser.add_argument(
        'bagfile', type=str, help='path to a bagfile which should be merged to the main bagfile')
    args = parser.parse_args()
    return args


def merge_bag(main_bagfile, bagfile, outfile):
    # get min and max time in bagfile

    # check output file

    if outfile == None:
        pattern = main_bagfile + "_merged_%i.bag"
        outfile = main_bagfile + "_merged.bag"
        shutil.copyfile(main_bagfile, outfile)
    # output some information
  #  print "merge bag %s in %s"%(bagfile, main_bagfile)
   # print "writing to %s."%outfile

    with rosbag.Bag(main_bagfile, 'r') as bag:
        start_time, end_time = get_limits(bag)

    # merge bagfile

    with rosbag.Bag(outfile, 'a') as outbag:
        for topic, msg, t in rosbag.Bag(bagfile).read_messages([], start_time=rospy.Time.from_sec(start_time), end_time=rospy.Time.from_sec(end_time)):
            outbag.write(topic, msg, t)


def get_limits(bag):
    start_time = bag.get_start_time()
    end_time = bag.get_end_time()
    print(start_time)
    print(end_time)
    return (start_time, end_time)


if __name__ == "__main__":
    args = parse_args()
    if args.t != None:
        args.t = args.t.split(',')
    merge_bag(args.main_bagfile,
              args.bagfile,
              outfile=args.o)
