#!/usr/bin/env python

########################################################
#
# Simple interface for spawning and interacting with models in gazebo.
# Also includes functionality for spawning and reading from a simulated camera sensor.
#
# Based on https://github.com/GraspDeepLearning/generate_gazebo_output
#######################################################

import rospy
from rospkg import RosPack
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from gazebo_ros import gazebo_interface
from gazebo_msgs.srv import (DeleteModelRequest, DeleteModelResponse, DeleteModel, 
                             GetModelState, GetModelStateRequest, GetWorldProperties,
                             SetModelState, SetModelStateRequest)

import numpy as np
from time import sleep
from os import environ, listdir
from os.path import isdir

import tf_conversions
#import PyKDL
import math
import std_srvs.srv

#TODO Debug only, remove later
import rospy 
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2


class RGBDListener():

    def __init__(self,
                 depth_topic="/camera1/camera/depth/image_raw",
                 points_topic="/camera1/camera/depth/points",
                 rgb_topic="/camera1/camera/rgb/image_raw",
                 info_topic="/camera1/camera/rgb/camera_info"):

        self.points_topic = points_topic
        self.depth_topic = depth_topic
        self.rgb_topic = rgb_topic
        self.info_topic = info_topic
        self.initialized = False
        # TODO Determine img dim from CameraInfo message:
        self.cam_info = None
        self.rgbd_image = np.zeros((480,640, 4)) 
        # Also save img timestamps:
        self.rgb_stamp = None
        self.depth_stamp = None
        self.points = None

    def get_cam_info(self):
        if self.cam_info is None:
            print "waiting for cam info"
            self.cam_info = rospy.wait_for_message(self.info_topic, 
                                                   CameraInfo)
        return self.cam_info

    def points_callback(self, data):
        self.points = data

    def depth_image_callback(self, data):
        #if self.initialized:
        depth_image_np = self.image2numpy(data)
        depth_image_np = np.nan_to_num(depth_image_np)
        self.rgbd_image[:, :, 3] = depth_image_np
        self.depth_stamp = data.header.stamp
        #else:
        #   print "Skipped a depth img."

    def rgb_image_callback(self, data):
        rgbd_image_np = self.image2numpy(data)
        self.rgbd_image[:, :, 0:3] = rgbd_image_np
        self.rgb_stamp = data.header.stamp


    def depth_from_points(self):
        if self.points is None:
            print "No points received yet.  Waiting for points."
            self.points = rospy.wait_for_message(self.points_topic, 
                                                 PointCloud2, timeout=5)
        points = point_cloud2.read_points(self.points)
        arry = np.array([p[2] for p in points])
        arry = np.reshape(arry,(480,640))
        arry = np.nan_to_num(arry)
        return arry

    #this method from:
    #https://github.com/rll/sushichallenge/blob/master/python/brett2/ros_utils.py
    def image2numpy(self, image):
        if image.encoding == 'rgb8':
            return np.fromstring(image.data, dtype=np.uint8).reshape(image.height, image.width, 3)[:, :, ::-1]
        if image.encoding == 'bgr8':
            return np.fromstring(image.data, dtype=np.uint8).reshape(image.height, image.width, 3)
        elif image.encoding == 'mono8':
            return np.fromstring(image.data, dtype=np.uint8).reshape(image.height, image.width)
        elif image.encoding == '32FC1':
            return np.fromstring(image.data, dtype=np.float32).reshape(image.height, image.width)
        else:
            raise Exception

    def listen(self):
        #rospy.init_node('listener', anonymous=True)

        rospy.Subscriber(self.points_topic, PointCloud2, self.points_callback, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self.depth_image_callback, queue_size=1)
        rospy.Subscriber(self.rgb_topic, Image, self.rgb_image_callback, queue_size=1)

class GazeboCameraManager():
    def __init__(self, gazebo_namespace="/gazebo"):
        self.gazebo_namespace = gazebo_namespace
        self.cam_listener = RGBDListener()
        self.cam_listener.listen()
        self.camera_name = "camera1"

        self.get_model_state_service = rospy.ServiceProxy(gazebo_namespace + '/get_model_state', GetModelState)
        self.set_model_state_service = rospy.ServiceProxy(gazebo_namespace + '/set_model_state', SetModelState)


    def spawn_camera(self):
        # parameter name MUST be "robot_description"
        model_xml = rospy.get_param("robot_description")

        #f = PyKDL.Frame(PyKDL.Rotation.RPY(0, math.pi+math.pi/4.0, math.pi), PyKDL.Vector(0, 0, 2))
        #f = PyKDL.Frame(PyKDL.Rotation.RPY(0, math.pi/4.0, 0), PyKDL.Vector(0, 0, 2))
        #model_pose = tf_conversions.posemath.toMsg(f)
        model_pose = Pose()
        model_pose.orientation.w = 1
        robot_namespace = self.camera_name
        gazebo_interface.spawn_urdf_model_client(model_name=self.camera_name,
                                                model_xml=model_xml,
                                                robot_namespace=robot_namespace,
                                                reference_frame="world",
                                                gazebo_namespace=self.gazebo_namespace,
                                                initial_pose=model_pose)

    def get_cam_info(self):
        return self.cam_listener.get_cam_info()

    def get_RGBD(self):
        image = self.cam_listener.rgbd_image
        
        while image[:, :, 3].max() == 0.0:
            print "Depth image invalid.  Recovering depth from [cam]depth/points topic."
            sleep(.5)
            depth = None
            depth = self.cam_listener.depth_from_points()
            image[:,:,3] = depth
        return image

    def get_RGB(self):
        image = self.cam_listener.rgbd_image
        return image[:,:,0:3]

    def get_RGB_and_stamp(self):
        image = self.cam_listener.rgbd_image
        return image[:,:,0:3], self.cam_listener.rgb_stamp

    def get_model_state(self):
        get_model_state_req = GetModelStateRequest()
        get_model_state_req.model_name = self.camera_name
        return self.get_model_state_service(get_model_state_req)

    def set_model_state(self, pose=Pose()):
        set_model_state_req = SetModelStateRequest()
        set_model_state_req.model_state.model_name = self.camera_name
        set_model_state_req.model_state.pose = pose
        return self.set_model_state_service(set_model_state_req)



class GazeboModelManager():

    def __init__(self,
                 gazebo_namespace="/gazebo"):

        self.gazebo_namespace = gazebo_namespace
        self.delete_model_service_proxy = rospy.ServiceProxy(gazebo_namespace + '/delete_model', DeleteModel)
        self.get_model_state_service_proxy = rospy.ServiceProxy(gazebo_namespace + '/get_model_state', GetModelState)
        self.set_model_state_service_proxy = rospy.ServiceProxy(gazebo_namespace + '/set_model_state', SetModelState)
        self.get_world_properties_proxy = rospy.ServiceProxy(gazebo_namespace + '/get_world_properties', GetWorldProperties)
        self.pause_physics_service_proxy = rospy.ServiceProxy(gazebo_namespace + "/pause_physics", std_srvs.srv.Empty)
        self.unpause_physics_service_proxy = rospy.ServiceProxy(gazebo_namespace + "/unpause_physics", std_srvs.srv.Empty)

        # Save list of potential model locations:
        try:
            self.models_dirs = environ['GAZEBO_MODEL_PATH'].split(":")
        except KeyError:
            raise Exception("GAZEBO_MODEL_PATH not found in environment. Don't know where to look for models.")

    # Find the given model's parent dir in GAZEBO_MODEL_PATH.
    def find_model_dir(self, model_name):
        for d in self.models_dirs:
            print "Trying %s" % (d+"/"+model_name)
            if isdir(d+"/"+model_name):
                return d
        raise Exception("Could not find model %s in any dirs in GAZEBO_MODEL_PATH" % model_name)
    
    def remove_model(self, model_name="coke_can"):

        del_model_req = DeleteModelRequest(model_name)
        self.delete_model_service_proxy(del_model_req)

    def clear_world(self):
        world_properties = self.get_world_properties_proxy()
        for model_name in world_properties.model_names:
            if model_name != "camera1":
                self.remove_model(model_name)

    def pause_physics(self):
        self.pause_physics_service_proxy()

    def unpause_physics(self):
        self.unpause_physics_service_proxy()

    def spawn_model(self, model_name, model_type, model_pose=None):
        model_dir = self.find_model_dir(model_type)
        model_xml = open(model_dir + "/" + model_type + "/model.sdf").read()
        if not model_pose:
            model_pose = Pose()
            model_pose.position.x = 0
            model_pose.position.y = 0
            model_pose.position.z = 0
        robot_namespace = model_name
        print "spawning!"
        gazebo_interface.spawn_sdf_model_client(model_name=model_name,
                                                model_xml=model_xml,
                                                robot_namespace=robot_namespace,
                                                initial_pose=model_pose,
                                                reference_frame="world",
                                                gazebo_namespace=self.gazebo_namespace)

        #large models can take a moment to load
        while not self.does_world_contain_model(model_name):
            sleep(0.5)

    def spawn_test_box(self, box_name="0", x=0, y=0, z=0):

        rpk = RosPack()
        package_root = rpk.get_path("grasp_dataset_generation")
        model_xml = open(package_root + "/urdf/test_box/model.sdf").read()
        model_pose = Pose()
        model_pose.position.x = x
        model_pose.position.y = y
        model_pose.position.z = z
        robot_namespace = box_name
        gazebo_interface.spawn_sdf_model_client(model_name=box_name,
                                                model_xml=model_xml,
                                                robot_namespace=robot_namespace,
                                                initial_pose=model_pose,
                                                reference_frame="world",
                                                gazebo_namespace=self.gazebo_namespace)

    def spawn_point_marker(self, marker_name="0", x=0, y=0, z=0):

        rpk = RosPack()
        package_root = rpk.get_path("grasp_dataset_generation")
        model_xml = open(package_root + "/urdf/point_marker/model.sdf").read()
        model_pose = Pose()
        model_pose.position.x = x
        model_pose.position.y = y
        model_pose.position.z = z
        robot_namespace = marker_name
        gazebo_interface.spawn_sdf_model_client(model_name=marker_name,
                                                model_xml=model_xml,
                                                robot_namespace=robot_namespace,
                                                initial_pose=model_pose,
                                                reference_frame="world",
                                                gazebo_namespace=self.gazebo_namespace)


    def does_world_contain_model(self, model_name):
        get_model_state_req = GetModelStateRequest()
        get_model_state_req.model_name = model_name
        resp = self.get_model_state_service_proxy(get_model_state_req)
        return resp.success

    def get_model_state(self, model_name):
        get_model_state_req = GetModelStateRequest()
        get_model_state_req.model_name = model_name
        return self.get_model_state_service_proxy(get_model_state_req)

    def set_model_state(self, model_name, pose=Pose()):
        set_model_state_req = SetModelStateRequest()
        set_model_state_req.model_state.model_name = model_name
        set_model_state_req.model_state.pose = pose
        return self.set_model_state_service_proxy(set_model_state_req)
