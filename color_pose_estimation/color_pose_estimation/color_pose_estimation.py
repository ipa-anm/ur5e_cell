import sys
import os
import message_filters
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import sensor_msgs.msg as sensor_msgs
import color_pose_estimation.registration as reg
import color_pose_estimation.detect_color as cdet
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np
import open3d as o3d
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point, Pose, TransformStamped
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
from tf2_ros.transform_listener import TransformListener
import image_geometry
import asyncio
from collections import namedtuple
import ctypes
import math
import struct
from sensor_msgs.msg import PointCloud2, PointField
from color_pose_msgs.msg import ColorPose
import ros2_numpy as rnp
import pyrealsense2 as rs2
from tf2_ros import TransformBroadcaster
import time

from rclpy.executors import MultiThreadedExecutor




# COLORTYPES

BLUE = 1
RED = 2
YELLOW = 3
WHITE = 4
BLACK = 5


class PCDListener(Node):
    def __init__(self):
        super().__init__('pcd_subscriber_node')
        self.color_pose = ColorPose()
        self.br = CvBridge()
        self.center = [0.0,0.0,0.0]
        self.publisher_ = self.create_publisher(
            ColorPose, '/color_pose_estimation/color_pose', 30)
        self.image_sub = Subscriber(
            self, sensor_msgs.Image, "/camera/color/image_raw", qos_profile=qos_profile_sensor_data)
        self.pcd_sub = Subscriber(self, sensor_msgs.PointCloud2, "/camera/depth/color/points", qos_profile=qos_profile_sensor_data)
        #self.camera_info_sub = Subscriber(self, sensor_msgs.CameraInfo, "/camera/color/camera_info")
        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.pcd_sub],10,0.1,)
        #node = Node('tf_listener')
      
        self.tf_broadcaster = TransformBroadcaster(self)
        self.ts.registerCallback(self.color_estimation_callback)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self, spin_thread=True)




    def color_estimation_callback(self, image, pcd):

        ######Image Processing#########

        # CV2 bridge to receive image in CV2 format
        self.current_frame = self.br.imgmsg_to_cv2(
            image, desired_encoding="bgr8")
        
        
        #cv2.imshow("image", self.current_frame)
        #key = cv2.waitKey(1)


        # ROS PointCloud2 to Numpy Array
        pc = rnp.numpify(pcd)
        points=np.zeros((pc.shape[0],3))
        points[:,0]=pc['x']
        points[:,1]=pc['y']
        points[:,2]=pc['z']
        p =np.array(points, dtype=np.float32)
        self.o3d_pcd = o3d.geometry.PointCloud()
        self.o3d_pcd.points = o3d.utility.Vector3dVector(p)
           
        # color detection node returns the bounding box of the found object in 
        # the bounding box format (x,y,w,h)
        rect = cdet.detect(self.current_frame)
        if (rect[2]<10): 
            return
        # Rescale the Pointcloud, as it is barely visible in Open3d Pointcloud Viewer
        self.o3d_pcd.scale(10, center=self.o3d_pcd.get_center())

        center_x_pcd, center_y_pcd, size = self.transform_coordinate_pixel_to_pointcloud_points(rect)


        # Define bounding box of object in Poincloud Coordinate system 
        center = np.array([center_x_pcd, center_y_pcd, 0.5])
        r = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        bbox = o3d.geometry.OrientedBoundingBox(center, r, size)

        # Visualize the bounding box and crop the pointcloud around the bounding box coordinates
        # o3d.visualization.draw_geometries([self.o3d_pcd, bbox],
        #                                   zoom=0.7,
        #                                   front=[0.5439, -0.2333, -0.8060],
        #                                   lookat=[2.4615, 2.1331, 1.338],
        #                                   up=[-0.1781, -0.9708, 0.1608])
        self.o3d_pcd = self.o3d_pcd.crop(bbox)
        self.o3d_pcd = self.o3d_pcd.translate((-center[0], -center[1],-0.25))

        o3d.io.write_point_cloud("copy_of_fragment.pcd", self.o3d_pcd)
        
        self.center = center
        
        # self.camera_model = image_geometry.PinholeCameraModel()
        # self.camera_info = camera_info

        # Pointcloud matching with the cropped bounding box 
        # returns a transformation matrix 4x4 consisting of 
        # the rotation and translation within the cropped area
        self.transformation_matrix = reg.register(self.o3d_pcd)

        # transform the received pose in Point Cloud Coordinate System to 
        # World coordinates and publish it as a color pose message

        self.transform_pose(self.transformation_matrix)

    


       


    def transform_coordinate_pixel_to_pointcloud_points(self, rect):

        x, y, w, h = rect
        # Estimate Size of the Pointcloud to convert pixel position to pointcloud position
        bbox = o3d.geometry.PointCloud.get_axis_aligned_bounding_box(self.o3d_pcd)
        size_pointcloud = np.asarray(o3d.geometry.AxisAlignedBoundingBox.get_box_points(bbox))

        # Convert center pixel of found bounding box to coordinate system in Pointcloud
        x_pointcloud = abs(size_pointcloud[0][0])
        y_pointcloud = abs(size_pointcloud[0][1])
        center_x_img = x + w/2 
        center_y_img = y + h/2
        coordinate_pc_x = (x_pointcloud*2)*0.01
        coordinate_pc_y = (y_pointcloud*2)*0.01
        center_x_pcd = (center_x_img/6.4) * coordinate_pc_x - x_pointcloud
        center_y_pcd = (center_y_img/4.8) * coordinate_pc_y - y_pointcloud
        size = np.array([w/6.4*coordinate_pc_x + 0.4, h/4.8*coordinate_pc_y + 0.4, h*0.03])

        return center_x_pcd, center_y_pcd, size 




    def transform_pose(self, transformation_matrix):
        # **Assuming /tf2 topic is being broadcasted
        pose_camera_frame = tf2_geometry_msgs.PoseStamped()

        #self.camera_model.fromCameraInfo(self.camera_info)

        #from_frame = self.camera_model.tfFrame()
        print(transformation_matrix[0][3], transformation_matrix[1][3])

        quaternion = self.matrix_to_quaternion(transformation_matrix)
        print(self.center[0], self.center[1], self.center[2])
        
        t = TransformStamped()
        now_stamp = self.get_clock().now()
        t.header.stamp = now_stamp.to_msg()
        t.header.frame_id = 'camera_depth_optical_frame'
        t.child_frame_id = 'box_frame'
        t.transform.translation.x = self.center[0]/10
        t.transform.translation.y = self.center[1]/10
        t.transform.translation.z = self.center[2]/10
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
    

        for i in range (0,10):
           self.tf_broadcaster.sendTransform(t)

        # print(self.color_pose)
    
    
    
    
    

    def matrix_to_quaternion(self, m):
        tr = m[0][0] + m[1][1] + m[2][2]

        if tr > 0:
            S = math.sqrt(tr+1.0) * 2
            # S = 4*qw
            qw = 0.25 * S
            qx = (m[2][1] - m[1][2]) / S
            qy = (m[0][2] - m[2][0]) / S
            qz = (m[1][0] - m[0][1]) / S
            return [qx, qy, qz, qw]

        elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
            S = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
            # S = 4*qx
            qw = (m[2][1] - m[1][2]) / S
            qx = 0.25 * S
            qy = (m[0][1] + m[1][0]) / S
            qz = (m[0][2] + m[2][0]) / S
            return [qx, qy, qz, qw]

        elif m[1][1] > m[2][2]:
            S = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
            # S = 4*qy
            qw = (m[0][2] - m[2][0]) / S
            qx = (m[0][1] + m[1][0]) / S
            qy = 0.25 * S
            qz = (m[1][2] + m[2][1]) / S
            return [qx, qy, qz, qw]

        else:
            S = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
            # S = 4*qz
            qw = (m[1][0] - m[0][1]) / S
            qx = (m[0][2] + m[2][0]) / S
            qy = (m[1][2] + m[2][1]) / S
            qz = 0.25 * S
            return [qx, qy, qz, qw]



def main(args=None):
    rclpy.init(args=args)
    pcd_listener = PCDListener()
    rclpy.spin(pcd_listener)
    pcd_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
