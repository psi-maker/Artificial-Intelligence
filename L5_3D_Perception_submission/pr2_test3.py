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
    print "pcl_callback is beginning"
    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)
    
    # TODO: Statistical Outlier Filtering
    #creating a noise outlier filter object
    fil = cloud.make_statistical_outlier_filter()
    fil.set_mean_k(50) # Set the number of neighboring points to analyze for any given point
    fil.set_std_dev_mul_thresh(0.01) # Set threshold scale factor, any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    # Finally call the filter function for magic
    cloud_filtered = fil.filter()

    # TODO: Voxel Grid Downsampling
    #Create a VoxelGrid filter object for our input point cloud
    vox = cloud_filtered.make_voxel_grid_filter()
    # Choose a voxel (also known as leaf) size, experiment and find the appropriate size!
    LEAF_SIZE = 0.008   
    # Set the voxel (or leaf) size  
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    # Call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()

    # TODO: PassThrough Filter
    # Create a PassThrough filter object.
    passthrough = cloud_filtered.make_passthrough_filter()
    # Assign axis and range to the passthrough filter object.
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)
    # Finally use the filter function to obtain the resultant point cloud. 
    cloud_filtered = passthrough.filter()

    passthrough = cloud_filtered.make_passthrough_filter()
    # Assign axis and range to the passthrough filter object.
    filter_axis = 'y'
    passthrough.set_filter_field_name(filter_axis)
    ayxis_min = -0.5
    ayxis_max = 0.5
    passthrough.set_filter_limits(ayxis_min, ayxis_max)
    # Finally use the filter function to obtain the resultant point cloud. 
    cloud_filtered = passthrough.filter()
    
     # TODO: RANSAC Plane Segmentation
    # Create the segmentation object
    seg = cloud_filtered.make_segmenter()
    # Set the model you wish to fit 
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    # Max distance for a point to be considered fitting the model, experiment with different values for max_distance for segmenting the table
    max_distance = 0.015
    seg.set_distance_threshold(max_distance)
    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    # TODO: Extract inliers and outliers
    cloud_filtered = cloud_filtered.extract(inliers, negative=True)

    # TODO: Euclidean Clustering to seperate mess object one by one
    # Euclidean Clustering to construct the k-d tree
    white_cloud = XYZRGB_to_XYZ(cloud_filtered)
    tree = white_cloud.make_kdtree()    
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(120)
    ec.set_MaxClusterSize(1500)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract() # contains a list of indices for each cluster (lists of points for each object)

    #Assign a color corresponding to each segmented object in scene
    #cluster_color = get_color_list(len(cluster_indices))
    #color_cluster_point_list = []
    #for j, indices in enumerate(cluster_indices):
    #    for i, indice in enumerate(indices):
    #        color_cluster_point_list.append([white_cloud[indice][0],
    #                                    white_cloud[indice][1],
    #                                    white_cloud[indice][2],
    #                                    rgb_to_float(cluster_color[j])])
    #Create new cloud containing all clusters, each with unique color
    #cluster_cloud = pcl.PointCloud_PointXYZRGB()
    #cluster_cloud.from_list(color_cluster_point_list)

    

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately

    # TODO: Convert PCL data to ROS messages

    # TODO: Publish ROS messages
    

    #filename = 'obj_filt.pcd'
    #pcl.save(cloud_filtered, filename)
    
    # Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    print "pcl_callback finish Euclidean clusting"
    detected_objects_labels = []
    detected_objects = []
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = cloud_filtered.extract(pts_list)        
        # TODO: convert the cluster from pcl to ROS using helper function
        ros_cluster_cloud = pcl_to_ros(pcl_cluster)
        # Compute the associated feature vector
        # Extract histogram features, complete this step just as is covered in capture_features.py
        chists = compute_color_histograms(ros_cluster_cloud, using_hsv=True)
        normals = get_normals(ros_cluster_cloud)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
        

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
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
        do.cloud = ros_cluster_cloud
        detected_objects.append(do)
    
    #rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

    # TODO: convert the cluster from pcl to ROS using helper function
    #ros_cluster_cloud = pcl_to_ros(cloud_filtered)

    # Publish the list of detected objects
    print "pcl_callback will publish info to ROS"
    pcl_camera_pub.publish(detected_objects)
    print "pcl_callback finish publishing info."
    

    # function to load parameters and request PickPlace service
def pr2_mover(object_list):
    
    # TODO: Initialize variables
    test_scene_num = Int32()
    test_scene_num.data = 1
    
    object_name = String()
    arm_name = String()
    pick_pose = Pose()
    place_pose = Pose()
    
    dict_list = []
    yaml_filename = 'output_3.yaml'
    
    
    labels = []
    centroids = [] # to be list of tuples (x, y, z)
    for obj in object_list:
        labels.append(obj.label)
        points_arr = ros_to_pcl(obj.cloud).to_array()
        centroids.append(np.mean(points_arr, axis=0)[:3])
    

    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')   
    dropbox_param = rospy.get_param('/dropbox')

    # TODO: Parse parameters into individual variables

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list
    for i in range(0, len(object_list_param)):

        object_name.data = object_list_param[i]['name']
        object_group = object_list_param[i]['group']

        # TODO: Get the PointCloud for a given object and obtain it's centroid
        for j in range(0, len(labels)):
            if object_name.data == labels[j]:
                pick_pose.position.x = np.asscalar(centroids[j][0])
                pick_pose.position.y = np.asscalar(centroids[j][1])
                pick_pose.position.z = np.asscalar(centroids[j][2])

        # TODO: Create 'place_pose' for the object
        for j in range(0, len(dropbox_param)):
            if object_group == dropbox_param[j]['group']:
                place_pose.position.x = dropbox_param[j]['position'][0]
                place_pose.position.y = dropbox_param[j]['position'][1]
                place_pose.position.z = dropbox_param[j]['position'][2]

        # TODO: Assign the arm to be used for pick_place
        if object_group == 'green':
            arm_name.data = 'right'
        elif object_group == 'red':
            arm_name.data = 'left'

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        dict_list.append(yaml_dict)
	# TODO: Output your request parameters into output yaml file
        send_to_yaml(yaml_filename, dict_list)

        # Wait for 'pick_place_routine' service to come up
        
        rospy.wait_for_service('pick_place_routine')
        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_camera_pub = rospy.Publisher("/pcl_camera", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
        

    # TODO: Load Model From disk
    model = pickle.load(open('model_test3.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()

