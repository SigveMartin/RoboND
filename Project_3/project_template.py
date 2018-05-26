#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    ## Convert ROS msg to PCL data
    pcl_cloud=ros_to_pcl(pcl_msg)

    ## Statistical Outlier Filtering
    # Much like the previous filters, we start by creating a filter object: 
    pcl_cloud = pcl_cloud.make_statistical_outlier_filter()

    # Set the number of neighboring points to analyze for any given point
    pcl_cloud.set_mean_k(50)

    # Set threshold scale factor
    x = 0.01

    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    pcl_cloud.set_std_dev_mul_thresh(x)

    # Finally call the filter function for magic
    cloud_outlier_filtered = pcl_cloud.filter()
    
    ## Voxel Grid Downsampling
    # Create a VoxelGrid filter object for our input point cloud
    vox = cloud_outlier_filtered.make_voxel_grid_filter()

    # Choose a voxel (also known as leaf) size
    LEAF_SIZE = 0.003

    # Set the voxel (or leaf) size  
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    # Call the filter function to obtain the resultant downsampled point cloud
    cloud_vox_filtered = vox.filter()

    ## PassThrough Filter

    # Create a PassThrough filter object for z axis.
    passthrough_z = cloud_vox_filtered.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    # along z - for table top
    filter_axis = 'z'
    passthrough_z.set_filter_field_name(filter_axis)
    axis_min = 0.61
    axis_max = 1.1
    passthrough_z.set_filter_limits(axis_min, axis_max)
    # Finally use the filter function to obtain the resultant point cloud. 
    cloud_ptz_filtered = passthrough_z.filter()
    
    # Create a PassThrough filter object for y axis.
    passthrough_y = cloud_ptz_filtered.make_passthrough_filter()
    # along y - for field of view to get rid of dropboxes
    filter_axis2 = 'y'
    passthrough_y.set_filter_field_name(filter_axis2)
    axis_min = -0.4
    axis_max = 0.4
    passthrough_y.set_filter_limits(axis_min, axis_max)
    # Finally use the filter function to obtain the resultant point cloud. 
    cloud_ptzy_filtered = passthrough_y.filter()
    
    # Create a PassThrough filter object for x axis.
    passthrough_x = cloud_ptzy_filtered.make_passthrough_filter()
    # along y - for field of view to get rid of dropboxes
    filter_axis2 = 'x'
    passthrough_x.set_filter_field_name(filter_axis2)
    axis_min = 0.35
    axis_max = 0.90
    passthrough_x.set_filter_limits(axis_min, axis_max)
    # Finally use the filter function to obtain the resultant point cloud. 
    cloud_pt_filtered = passthrough_x.filter()
    
    ## RANSAC Plane Segmentation
    # Create the segmentation object
    seg = cloud_pt_filtered.make_segmenter()

    # Set the model you wish to fit 
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    # Experiment with different values for max_distance 
    # for segmenting the table
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)

    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    ## Extract inliers and outliers
    extracted_inliers = cloud_pt_filtered.extract(inliers, negative=False)
    extracted_outliers = cloud_pt_filtered.extract(inliers, negative=True)

    ## Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(extracted_outliers)
    tree = white_cloud.make_kdtree()

    ## Create Cluster-Mask Point Cloud to visualize each cluster separately
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.01)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(5000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()    

    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                        white_cloud[indice][1],
                                        white_cloud[indice][2],
                                         rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    ## Convert PCL data to ROS messages
    ros_filtered_outliers_cloud = pcl_to_ros(cloud_outlier_filtered)
    ros_pt_filtered_cloud = pcl_to_ros(cloud_pt_filtered)
    ros_vox_filtered_cloud = pcl_to_ros(cloud_vox_filtered)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    ros_cloud_objects = pcl_to_ros(extracted_outliers)
    ros_cloud_table = pcl_to_ros(extracted_inliers)

    ## Publish ROS messages
    pcl_filtered_outliers_pub.publish(ros_filtered_outliers_cloud)
    pcl_pt_filt_pub.publish(ros_pt_filtered_cloud)
    pcl_vox_filt_pub.publish(ros_vox_filtered_cloud)
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

    # Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = extracted_outliers.extract(pts_list)
        ## convert the cluster from pcl to ROS using helper function
        ros_cluster = pcl_to_ros(pcl_cluster)
        # Compute the associated feature vector
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)
        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    # Publish the list of detected objects
    # This is the output you'll need to complete the upcoming project!
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    object_list_param = rospy.get_param('/object_list')
    pick_list = [entry["name"] for entry in object_list_param] 

    if set(pick_list) == set(detected_objects_labels):
        try:
            pr2_mover(detected_objects)
        except rospy.ROSInterruptException:
            pass
    else:
	print("did not move pick list {} and detected labels {}".format(set(pick_list),set(detected_objects_labels)))

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # Initialize variables
    TEST_SCENE_NUM 	= Int32()
    # change num per world/picklist loaded 1, 2 or 3. 
    TEST_SCENE_NUM.data = 1
    yaml_filename = "output_%s.yaml"%(TEST_SCENE_NUM.data)

    # Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    db_param = rospy.get_param('/dropbox')
    
    left_db = \
	{"name"		: db_param[0]["name"],
	 "group"	: db_param[0]["group"],
	 "position"	: db_param[0]["position"]}
    right_db = \
	{"name"		: db_param[1]["name"],
	 "group"	: db_param[1]["group"],
	 "position"	: db_param[1]["position"]}
    
    # TODO: Rotate PR2 in place to capture side tables for the collision map

    dict_list = []
    #  Loop through the pick list
    for i in range(0,len(object_list_param)):
	labels = []
	centroids = [] # to be list of tuples (x, y, z)
        #  Get the PointCloud for a given object and obtain it's centroid
	for object in object_list:
    	    labels.append(object.label)
    	    points_arr = ros_to_pcl(object.cloud).to_array()
    	    centroids.append(np.mean(points_arr, axis=0)[:3])
	
	OBJECT_NAME = String()
	WHICH_ARM = String()
	PICK_POSE = Pose()
	PLACE_POSE = Pose()

	# Populate the data field
	OBJECT_NAME.data = object_list_param[i]['name']
	object_group = object_list_param[i]['group']

	# Get the pick_pose of the current object in picklist, if detected.
	print("Getting the %s objects pick pose"%OBJECT_NAME.data)
	idx = labels.index(OBJECT_NAME.data)
	PICK_POSE.position.x = np.asscalar(centroids[idx][0])
	PICK_POSE.position.y = np.asscalar(centroids[idx][1])
	PICK_POSE.position.z = np.asscalar(centroids[idx][1])
	print(PICK_POSE.position.x,PICK_POSE.position.y,PICK_POSE.position.z)

        #  Create 'place_pose' for the object
	#  Assign the arm to be used for pick_place
	if object_group == right_db["group"]:
	    WHICH_ARM.data = right_db["name"]
	    PLACE_POSE.position.x = right_db["position"][0]
	    PLACE_POSE.position.y = right_db["position"][1]
	    PLACE_POSE.position.z = right_db["position"][2]
	else:
	    WHICH_ARM.data = left_db["name"]
	    PLACE_POSE.position.x = left_db["position"][0]
	    PLACE_POSE.position.y = left_db["position"][1]
	    PLACE_POSE.position.z = left_db["position"][2]

        #  Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
	yaml_dict = make_yaml_dict(TEST_SCENE_NUM, WHICH_ARM, OBJECT_NAME, PICK_POSE, PLACE_POSE)
        dict_list.append(yaml_dict)
	
        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # Insert your message variables to be sent as a service request
            resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)
   
            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # Output your request parameters into output yaml file
    send_to_yaml(yaml_filename, dict_list)


if __name__ == '__main__':

    ## ROS node initialization
    rospy.init_node('clustering', anonymous=True)
    ## Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)
    ## Create Publishers
    pcl_filtered_outliers_pub = rospy.Publisher("/pcl_filtered_outliers", PointCloud2, queue_size=1)
    pcl_pt_filt_pub = rospy.Publisher("/pcl_pt", PointCloud2, queue_size=1)
    pcl_vox_filt_pub = rospy.Publisher("/pcl_vox", PointCloud2, queue_size=1)
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)
    ## Load Model From disk
    model = pickle.load(open('model_pl1.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
    # Initialize color_list
    get_color_list.color_list = []

    ## Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
