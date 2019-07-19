#!/usr/bin/env python

import rospy

from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import PointCloud2


__author__ = 'aGn'

PATH_TO_SAVE = rospy.get_param('/object_tracker/path_to_save')
MARKER_TOPIC = rospy.get_param('/object_tracker/marker_topic')


class PoseExporter(object):
    def __init__(self):
        self.frame = 0
        print("The .csv files store in " + PATH_TO_SAVE)
        rospy.Subscriber(MARKER_TOPIC, MarkerArray, self.export_pose, queue_size=1)
        rospy.Subscriber("/velodyne_points", PointCloud2, self.get_frame, queue_size=1)

    def get_frame(self, pcl):
        self.frame += 1

    def export_pose(self, marker):
        # print(marker.markers)
        for mr in marker.markers:
            if mr.pose.position.x > 0.1 and (mr.pose.position.x != 0 and mr.pose.position.y != 0):
                '''Linear Person'''

                with open(PATH_TO_SAVE + 'did_pedestrian_linear.csv', mode='a') as file_:
                    file_.write("{},{},{}".format(self.frame,
                                                  mr.pose.position.x,
                                                  mr.pose.position.y))
                    file_.write("\n")

            elif mr.pose.position.x != 0 and mr.pose.position.y != 0:
                '''Rotary person'''

                with open(PATH_TO_SAVE + 'didi_pedestrian_rotary.csv', mode='a') as file_:
                    file_.write("{},{},{}".format(self.frame,
                                                  mr.pose.position.x,
                                                  mr.pose.position.y))
                    file_.write("\n")


if __name__ == "__main__":
    rospy.init_node('object_tracker', anonymous=True)
    PoseExporter()
    rospy.spin()
