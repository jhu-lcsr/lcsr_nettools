#!/usr/bin/env python

import roslib; roslib.load_manifest('lcsr_nettools')

import re

import rospy
import tf

import tf.msg as tf_msgs
import lcsr_nettools.msg as lcsr_nettools_msgs


class TFSamplePub(object):
    """Sample from a TF message and re-publish the samples.
    
    Publishes HeaderSample messages on "statistics/tf_frames/%frame_id%/header_samples"
    """
    def __init__(self, frame_id):
        # Store the frame id
        self.frame_id = frame_id

        # Remove front slashes
        frontslash_regex = re.compile(r"^\/+")
        frame_id = frontslash_regex.sub('',frame_id)

        # Create a publisher to publish the samples
        self.sample_publisher = rospy.Publisher('statistics/tf_frames/%s/header_samples' % frame_id, lcsr_nettools_msgs.HeaderSample)

    def tf_msg_cb(self, msg):
        # Get the receive time
        stamp = rospy.get_rostime()

        # Pull out the frame we want, and publish a header
        for header in [f.header for f in msg.transforms if f.child_frame_id == self.frame_id]:
            sample = lcsr_nettools_msgs.HeaderSample(header, stamp)
            self.sample_publisher.publish(sample)

def main():
    rospy.init_node('tf_statistics')

    # Get the frame id we want to track statistics of
    frame_id = rospy.get_param('~frame_id')

    sample_pub = TFSamplePub(frame_id)
    tf_sub = rospy.Subscriber('tf',tf_msgs.tfMessage,sample_pub.tf_msg_cb)

    rospy.spin()

if __name__ == '__main__':
    main()

