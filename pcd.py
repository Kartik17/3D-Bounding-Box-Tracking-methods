#!/usr/bin/env python

import rospy
import pcl
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField, PointCloud
#import sensor_msgs.point_cloud2 as pc2
from sensor_msgs import point_cloud2
import struct
import ctypes
from random import randint

def ros_to_pcl(ros_cloud):
    points_list = []

    for data in point_cloud2.read_points(ros_cloud, skip_nans=True):
        points_list.append([data[0], data[1], data[2], data[3]])

    pcl_data = pcl.PointCloud_PointXYZRGB()
    pcl_data.from_list(points_list)

    return pcl_data

def pcl_to_ros(pcl_array): 
    ros_msg = PointCloud2()

    ros_msg.header.stamp = rospy.Time.now()
    ros_msg.header.frame_id = "pandar"
    ros_msg.height = 1
    ros_msg.width = pcl_array.size

    ros_msg.fields.append(PointField(
                            name="x",
                            offset=0,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="y",
                            offset=4,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="z",
                            offset=8,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="i",
                            offset=12,
                            datatype=PointField.FLOAT32, count=1))
  


    ros_msg.is_bigendian = False
    ros_msg.point_step = 12
    ros_msg.row_step = ros_msg.point_step * ros_msg.width * ros_msg.height
    ros_msg.is_dense = False
    buffer = []

    for data in pcl_array:
    	float_rgb = struct.unpack('f', struct.pack('i', 0xff0000))[0]
    	#print(data[3])
    	try:
        	buffer.append(struct.pack('fff', data[0], data[1], data[2]))
        except Exception as e:
        	print(e)
        else:
        	pass
    ros_msg.data = "".join(buffer)

    return ros_msg


def pclxyzrgb_to_ros(pcl_array):
    ros_msg = PointCloud2()

    ros_msg.header.stamp = rospy.Time.now()
    ros_msg.header.frame_id = "pandar"

    ros_msg.height = 1
    ros_msg.width = pcl_array.size

    ros_msg.fields.append(PointField(
                            name="x",
                            offset=0,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="y",
                            offset=4,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="z",
                            offset=8,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="rgb",
                            offset=16,
                            datatype=PointField.FLOAT32, count=1))

    ros_msg.is_bigendian = False
    ros_msg.point_step = 32
    ros_msg.row_step = ros_msg.point_step * ros_msg.width * ros_msg.height
    ros_msg.is_dense = False
    buffer = []

    for data in pcl_array:
        s = struct.pack('>f', data[3])
        i = struct.unpack('>l', s)[0]
        pack = ctypes.c_uint32(i).value

        r = (pack & 0x00FF0000) >> 16
        g = (pack & 0x0000FF00) >> 8
        b = (pack & 0x000000FF)

        buffer.append(struct.pack('ffffBBBBIII', data[0], data[1], data[2], 1.0, b, g, r, 0, 0, 0, 0))

    ros_msg.data = "".join(buffer)

    return ros_msg

def to_XYZ(cloud):
	xyz_cloud = pcl.PointCloud()
	temp_list = []
	for data in cloud:
		temp_list.append([data[0],data[1],data[2]])
	xyz_cloud.from_list(temp_list)

	return xyz_cloud


def euclidean_clustering(cloud):
	#print(cloud.height)
	cloud = ros_to_pcl(cloud)
	white_cloud = to_XYZ(cloud)
	tree = white_cloud.make_kdtree()

	ec = white_cloud.make_EuclideanClusterExtraction()
	ec.set_ClusterTolerance(0.5)
	ec.set_MinClusterSize(50)
	ec.set_MaxClusterSize(50000)

	ec.set_SearchMethod(tree)

	cluster_indices = ec.Extract()

	cluster_color = get_color_list(len(cluster_indices))
		
		#print('max_X:{},min_X:{},max_Y:{},min_Y:{},max_Z:{},min_Z:{}'.format(white_cloud[cluster_id]))

	#print(cluster_color)

	color_cluster_point_list = []

	for j, indices in enumerate(cluster_indices):
		for i, indice in enumerate(indices):
			color_cluster_point_list.append([white_cloud[indice][0],white_cloud[indice][1],white_cloud[indice][2], rgb_to_float(cluster_color[j])])

	cluster_cloud = pcl.PointCloud_PointXYZRGB()
	cluster_cloud.from_list(color_cluster_point_list)

	print(color_cluster_point_list[:5])
	ros_msg = pclxyzrgb_to_ros(cluster_cloud)
	print('Node_Running_Euclid')
	pcl_object_cluster_pub.publish(ros_msg)

	#return ros_msg



def get_color_list(cluster_count):
    color_list = []
    if (cluster_count > 0):
        for i in xrange(cluster_count):
            color_list.append(random_color_gen())
    return color_list

def random_color_gen():
    r = randint(0, 255)
    g = randint(0, 255)
    b = randint(0, 255)
    return [r, g, b]


def rgb_to_float(color):
    """ Converts an RGB list to the packed float format used by PCL
        From the PCL docs:
        "Due to historical reasons (PCL was first developed as a ROS package),
         the RGB information is packed into an integer and casted to a float"
        Args:
            color (list): 3-element list of integers [0-255,0-255,0-255]
        Returns:
            float_rgb: RGB value packed as a float
    """
    hex_r = (0xff & color[0]) << 16
    hex_g = (0xff & color[1]) << 8
    hex_b = (0xff & color[2])

    hex_rgb = hex_r | hex_g | hex_b

    float_rgb = struct.unpack('f', struct.pack('i', hex_rgb))[0]

    return float_rgb


def segment_the_ground(cloud,axis):

	seg = cloud.make_segmenter_normals(ksearch=10)
	seg.set_optimize_coefficients(True)
	seg.set_model_type(pcl.SACMODEL_PERPENDICULAR_PLANE)
	seg.set_method_type(pcl.SAC_RANSAC)
	seg.set_axis(0.0,0.0,1.0)
	seg.set_eps_angle(10*(np.pi/180.))
	seg.set_distance_threshold(0.4)
	#seg.set_normal_distance_weight(0.95)
	#w the relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point normals and the plane normal. (The Euclidean distance will have weight 1-w.)
	seg.set_max_iterations(10000)
	inliers, coefficients = seg.segment()


	#print(coefficients)
	cloud_object = cloud.extract(inliers,negative =True)
	cloud_road = cloud.extract(inliers,negative = False)

	return cloud_road, cloud_object

def filter_pointcloud(cloud):

	fil = cloud.make_statistical_outlier_filter()
	fil.set_mean_k(20)
	fil.set_std_dev_mul_thresh(1.0)

	return fil.filter()

def filter_method(cloud, center, dimensions):
	max_dimension = max(dimensions)

	for axis, value in center.items():
		passthrough = cloud.make_passthrough_filter()

		passthrough.set_filter_field_name (axis)
		passthrough.set_filter_limits (-max_dimension*0.55 + value, max_dimension*0.55 + value)
		cloud_filtered = passthrough.filter ()

		cloud = cloud_filtered

	return cloud_filtered

def search_two(cloud_array,center,dimensions):
	max_dimension = max(dimensions)

	min_x = center['x'] - 1*max_dimension
	max_x = center['x'] + 1*max_dimension
	                
	min_y = center['y'] - 1*max_dimension
	max_y = center['y'] + 1*max_dimension

	x_filtered_points = [a for a in cloud_array if (a[0] >= min_x and a[0] <= max_x)]
	y_filtered_points = [a for a in x_filtered_points if (a[1] >= min_y and a[1] <= max_y)]

	return y_filtered_points


if __name__ == '__main__':
	
	rospy.init_node('pointcloud', anonymous = True)
	pcl_pub = rospy.Publisher("/pcl_full_scene",PointCloud2,queue_size=10)
	pcl_pub_2 = rospy.Publisher("/pcl_full_scene_not_rotated",PointCloud2,queue_size=10)
	pcl_road_pub = rospy.Publisher("/pcl_road", PointCloud2, queue_size = 10)
	pcl_object_pub = rospy.Publisher("/pcl_objects",PointCloud2,queue_size = 10)
	pcl_object_cluster_pub = rospy.Publisher("/pcl_cluster_objects", PointCloud2, queue_size =10)
	pcl_filtered_pub = rospy.Publisher('/pcl_filtered', PointCloud2, queue_size = 10)
	pcl_passthrough_filter_pub = rospy.Publisher('/passthrough', PointCloud2, queue_size = 10)
	pcl_passthrough_filter_n_pub = rospy.Publisher('/passthrough_n',PointCloud2,queue_size = 10)

	#p,p1 = pcl.PointCloud(),pcl.PointCloud()
	#p_load = pcl.load_XYZI("/home/kartik/pcd_files/zf/006049.pcd")
	#p.from_file("/home/kartik/pcd_files/zf/006049.pcd")



	#center = {'x':12.943, 'y':2.062,'z': -0.5}
	#dimensions = [6.75,2.643,2.295]
	#p_passthrough = filter_method(p,center,dimensions)

	##cloud_array = np.asarray(p)
	#y_filtered_points = search_two(cloud_array,center,dimensions)
	#p_passthrough_n = pcl.PointCloud(y_filtered_points)


	#p_filtered = filter_pointcloud(p)
	#pcl_road, pcl_object = segment_the_ground(p_filtered,'z')

	#ros_pointcloud = pcl_to_ros(p_load)
	#ros_pointcloud_not_rotated = pcl_to_ros(p1)
	ros_object_cluster_pointcoud = rospy.Subscriber("pandar_points", PointCloud2, euclidean_clustering)
	#ros_filtered = pcl_to_ros(p_filtered)
	#ros_road_pointcoud = pcl_to_ros(pcl_road)
	#ros_object_pointcoud = pcl_to_ros(pcl_object)
	#ros_object_cluster_pointcoud = euclidean_clustering(pcl_object)
	#ros_passthrough_cloud = pcl_to_ros(p_passthrough)
	#ros_passthrough_cloud_n = pcl_to_ros(p_passthrough_n)

	rate = rospy.Rate(5)

	while not rospy.is_shutdown():
		print("Node_Running")
		#pcl_object_pub.publish(ros_object_pointcoud)
		

		#pcl_pub.publish(ros_pointcloud)
		#pcl_pub_2.publish(ros_pointcloud_not_rotated)
		#pcl_filtered_pub.publish(ros_filtered)
		#pcl_road_pub.publish(ros_road_pointcoud)
		#pcl_object_pub.publish(ros_object_pointcoud)
		#pcl_object_cluster_pub.publish(ros_object_cluster_pointcoud)
		#pcl_passthrough_filter_pub.publish(ros_passthrough_cloud)
		#pcl_passthrough_filter_n_pub.publish(ros_passthrough_cloud_n)

		rate.sleep()

