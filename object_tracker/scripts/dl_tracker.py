#!/usr/bin/env python
import sys
import os
import time
import rospy as rp
import numpy as np
import math
import struct

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Float32MultiArray, Float64MultiArray, MultiArrayDimension
from visualization_msgs.msg import Marker, MarkerArray

from sklearn.cluster import DBSCAN

import tensorflow as tf
import keras
from keras.models import load_model
from keras.optimizers import Adam

from full_view_model import fcn_model
from full_view_train import *
from convert_to_full_view_panorama import *
from dl_filter import dl_filter

from keras.utils.generic_utils import get_custom_objects
get_custom_objects().update({"my_loss": my_loss})

PI = 3.14159265358979
PI_2 = 1.570796326794895
MAX_MARKER_COUNT = 30
PUBLISH_MARKERS = True
CAR_LABEL = 1

class dl_tracker:

	def __init__(self):
		# model
		dir_path = os.path.dirname(os.path.realpath(__file__))
		self.model = load_model(os.path.join(dir_path, '../model/fv_model_for_car_June_30_132_63.h5'))
		#self.model = load_model(os.path.join(dir_path, '../model/fv_July_01_18.h5'))
		# filter
		self.filter = dl_filter()
		# graph
		self.graph = tf.get_default_graph()
		# communication
		self.initialize_communication()
		rp.loginfo("dl_tracker: initialized")
		print "dl_tracker: initialized"

	def initialize_communication(self):
		self.subscriber = rp.Subscriber("/filtered_points", Float64MultiArray, self.on_points_received, queue_size=1)
		self.detected_marker_publisher = rp.Publisher("/tracker/markers/detect", MarkerArray, queue_size=1)
		self.predicted_marker_publisher = rp.Publisher("/tracker/markers/predict", MarkerArray, queue_size=1)
		self.box_publisher = rp.Publisher("/tracker/boxes", Float32MultiArray, queue_size=1)
		self.detected_markers = MarkerArray()
		self.predicted_markers = MarkerArray()
		for i in range(MAX_MARKER_COUNT):
			marker = Marker()
			marker.id = i
			marker.header.frame_id = "velodyne"
			marker.type = marker.CUBE
			marker.action = marker.ADD
			marker.pose.position.x = 0.0
			marker.pose.position.y = 0.0
			marker.pose.position.z = 0.0
			marker.pose.orientation.x = 0.0
			marker.pose.orientation.y = 0.0
			marker.pose.orientation.z = 0.0
			marker.pose.orientation.w = 1.0
			marker.scale.x = 1.0
			marker.scale.y = 1.0
			marker.scale.z = 1.0
			marker.color.a = 0.0
			marker.color.r = 1.0
			marker.color.g = 0.0
			marker.color.b = 0.0
			self.detected_markers.markers.append(marker)
		for i in range(1):
			marker = Marker()
			marker.id = i
			marker.header.frame_id = "velodyne"
			marker.type = marker.CUBE
			marker.action = marker.ADD
			marker.pose.position.x = 0.0
			marker.pose.position.y = 0.0
			marker.pose.position.z = 0.0
			marker.pose.orientation.x = 0.0
			marker.pose.orientation.y = 0.0
			marker.pose.orientation.z = 0.0
			marker.pose.orientation.w = 1.0
			marker.scale.x = 1.0
			marker.scale.y = 1.0
			marker.scale.z = 1.0
			marker.color.a = 0.0
			marker.color.r = 1.0
			marker.color.g = 1.0
			marker.color.b = 0.0
			self.predicted_markers.markers.append(marker)

	def on_points_received(self, data):
		total_start = time.time()

		ts_sec = int(data.data[0])
		ts_nsec = int(data.data[1])
		lidar = np.asarray(data.data[2:]).reshape(-1, 3)

		detected_boxes = np.empty((0,8))

		# predict
		with self.graph.as_default():
			detected_boxes = detect(self.model, lidar, clusterPoint=False, seg_thres=0.15, multi_box=False)

		# filter by velocity
		boxes = self.filter.filter_by_velocity(detected_boxes, ts_sec, ts_nsec)

		if PUBLISH_MARKERS:
			if len(detected_boxes) > 0:			
				self.publish_markers(detected_boxes, boxes)

		if len(boxes) > 0:			
			self.publish_detected_boxes(boxes)

		print ("total time: " + str(time.time() - total_start))

	def publish_detected_boxes(self, box_info):
		arr = Float32MultiArray()
		flat_box_info = np.reshape(box_info, (-1))
		arr.data = flat_box_info.tolist()
		# publish
		self.box_publisher.publish(arr)
		#rp.loginfo("dl_tracker: published %d boxes", len(box_info))

	def publish_markers(self, det_box_info, box_info):
		num_boxes = len(det_box_info)
		# update markers
		num_markers = min(num_boxes, MAX_MARKER_COUNT)
		for i in range(num_markers):
			info = det_box_info[i]
			marker = self.detected_markers.markers[i]
			marker.pose.position.x = info[1]
			marker.pose.position.y = info[2]
			marker.pose.position.z = info[3]
			marker.scale.x = info[4]
			marker.scale.y = info[5]
			marker.scale.z = info[6]
			marker.pose.orientation.z = info[7]			
			marker.color.a = 0.3
		# hide markers not used
		if num_boxes < MAX_MARKER_COUNT:
			for i in range(num_boxes, MAX_MARKER_COUNT):
				marker = self.detected_markers.markers[i]
				marker.color.a = 0.0
		if len(box_info) > 0:
			for i in range(1):
				info = box_info[i]
				marker = self.predicted_markers.markers[i]
				marker.pose.position.x = info[1]
				marker.pose.position.y = info[2]
				marker.pose.position.z = info[3]
				marker.scale.x = info[4]
				marker.scale.y = info[5]
				marker.scale.z = info[6]
				marker.pose.orientation.z = info[7]	
				marker.color.a = 0.3
		# publish
		self.detected_marker_publisher.publish(self.detected_markers)
		self.predicted_marker_publisher.publish(self.predicted_markers)		
		rp.loginfo("dl_tracker: published %d markers", num_markers)


def one_box_clustering(boxes, eps = 1, min_samples = 1):
	# Extract the center from predicted boxes
	box_centers = boxes[:,1:4]
	# Do clustering
	db = DBSCAN(eps=eps, min_samples=min_samples).fit(box_centers)
	labels = db.labels_

	n_clusters = len(set(labels))
	n_points = [np.sum([labels == i]) for i in range(n_clusters) ]
	max_point_cluster = np.argmax(n_points)

	index = (labels == max_point_cluster)
	box = np.mean(boxes[index], axis = 0)
	return np.expand_dims(box, 0)

def multi_box_clustering(boxes, eps = 1, min_samples = 1):
	# Extract the center from predicted boxes
	box_centers = boxes[:,1:4]
	# Do clustering
	db = DBSCAN(eps=eps, min_samples=min_samples).fit(box_centers)
	labels = db.labels_

	n_clusters = len(set(labels))
	mul_clusters = np.array([np.mean(boxes[labels == i], axis = 0) for i in range(n_clusters)])

	return mul_clusters

def detect(model, lidar, clusterPoint=True, cluster=True, seg_thres=0.5, multi_box=True):
	test_view =  fv_cylindrical_projection_for_test(lidar, clustering=clusterPoint)
	view = test_view[:,:,[5,2]].reshape(1,16,320,2)
	test_view_reshape = test_view.reshape(-1,6)

	pred = model.predict(view)

	pred = pred[0].reshape(-1,8)
	thres_pred = pred[pred[:,0] > seg_thres]
	thres_view = test_view_reshape[pred[:,0] > seg_thres]

	num_boxes = len(thres_pred)
	if num_boxes == 0:
		return np.array([])
	boxes = np.zeros((num_boxes,8))

	theta = thres_view[:,[3]]
	phi = thres_pred[:,[-1]]

	min = thres_view[:,:3] - rotation_v(theta, thres_pred[:,1:4]) # 0: left top
	max = thres_view[:,:3] - rotation_v(theta, thres_pred[:,4:7]) # 6: right bottom
	center = (min + max) * 0.5
	dvec = max - min
	sinphi = np.sin(phi)
	cosphi = np.cos(phi)
	normdxy = np.linalg.norm(dvec[:,:2], axis=1) # distance between 0 and 2
	normdxy = normdxy.reshape(-1, 1)
	width = normdxy * abs(sinphi)
	height = normdxy * abs(cosphi)
	depth = dvec[:,[2]]
	ax = np.arctan2(dvec[:,[1]], dvec[:,[0]]) # angle from x axis to vector 2-0
	rz = [0.5 * PI - phi[i] + ax[i] for i in range(len(ax))]

	boxes[:,[0]] = CAR_LABEL
	boxes[:,1:4] = center
	boxes[:,[4]] = width
	boxes[:,[5]] = height
	boxes[:,[6]] = depth
	boxes[:,[7]] = rz

	if not cluster:
		return boxes
	elif multi_box:
		mul_clusters = multi_box_clustering(boxes)
		return mul_clusters
	else:
		one_cluster = one_box_clustering(boxes)
		return one_cluster

def rotation_v(theta, points):
	v = np.sin(theta)
	u = np.cos(theta)
	out = np.copy(points)
	out[:,[0]] = u*points[:,[0]] + v*points[:,[1]]
	out[:,[1]] = -v*points[:,[0]] + u*points[:,[1]]
	return out

def listen():
	processor = dl_tracker()
	# In ROS, nodes are uniquely named. If two nodes with the same
	# node are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rp.init_node('dl_tracker', anonymous=True)
	# spin() simply keeps python from exiting until this node is stopped
	rp.spin()

if __name__ == '__main__':
	listen()