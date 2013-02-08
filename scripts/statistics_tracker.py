#!/usr/bin/env python

import roslib; roslib.load_manifest('lcsr_nettools')
import rostopic

import sys



def msg_handler(msg, tracker):
    try:
        header = msg.header
    except:
        rospy.logerr('Message type does not have a "header" field!')

def main():
    rospy.init_node('statistics_tracker')

    subscribers = []
    trackers = []

    # Create the subscribers
    subscribers +=  rospy.Subscriber(topic_name, rospy.AnyMsg


if __name__ == '__main__':
    main()
