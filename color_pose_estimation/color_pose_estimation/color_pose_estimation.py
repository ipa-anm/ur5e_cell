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
from geometry_msgs.msg import Point, Pose, TransformStamped, PoseStamped
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
from tf2_ros import TransformBroadcaster, TransformBroadcaster
import time
from tf2_geometry_msgs import do_transform_pose
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

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self, spin_thread=True)
        self.tf_broadcaster = TransformBroadcaster(self)

        #self.publisher_ = self.create_publisher(
        #    ColorPose, '/color_pose_estimation/color_pose', 30)
        self.image_sub = Subscriber(
            self, sensor_msgs.Image, "/camera/color/image_raw", qos_profile=qos_profile_sensor_data)
        self.pcd_sub = Subscriber(self, sensor_msgs.PointCloud2, "/camera/depth/color/points", qos_profile=qos_profile_sensor_data)
        self.aligned_depth_sub = Subscriber(self, sensor_msgs.Image, "/camera/aligned_depth_to_color/image_raw", qos_profile=qos_profile_sensor_data)

        self.camera_info_sub = Subscriber(self, sensor_msgs.CameraInfo, "/camera/aligned_depth_to_color/camera_info")
        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.pcd_sub, self.aligned_depth_sub, self.camera_info_sub],10,0.1,)
        #node = Node('tf_listener')
      
        self.ts.registerCallback(self.color_estimation_callback)

        self.publisher = self.create_publisher(tf2_geometry_msgs.PoseStamped, "/pose_topic", 10)



    def color_estimation_callback(self, image, pcd, depth, camera_info):

        ######Image Processing#########

        # CV2 bridge to receive image in CV2 format
        self.current_frame = self.br.imgmsg_to_cv2(
            image, desired_encoding="bgr8")
        print(self.current_frame.shape)
        
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

        depth_image = self.br.imgmsg_to_cv2(depth, "passthrough")
        depth_array = np.array(depth_image, dtype=np.float32)

        o3d_depth = o3d.geometry.Image(depth_array)
        o3d_rgb = o3d.geometry.Image(self.current_frame)

        try:
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_rgb, o3d_depth)
        except RuntimeError as e:
            print(f"Exception caught: {str(e)}")
            return
        
        self.fx = camera_info.k[0]
        self.fy = camera_info.k[4]
        self.cx = camera_info.k[2]
        self.cy = camera_info.k[5]
        intrinsic = o3d.camera.PinholeCameraIntrinsic(width=camera_info.width, height=camera_info.height, fx=self.fx, fy=self.fy, cx=self.cx, cy=self.cy)

        self.o3d_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic=intrinsic)
        # color detection node returns the bounding box of the found object in 
        # the bounding box format (x,y,w,h)
        rect = cdet.detect(self.current_frame)
        if (rect[2]<10): 
            return
        # Rescale the Pointcloud, as it is barely visible in Open3d Pointcloud Viewer
        #self.o3d_pcd.scale(10, center=self.o3d_pcd.get_center())

        center_image_x = int(rect[0]+(rect[2]/2))
        center_image_y = int(rect[1]+(rect[3]/2))
        print(center_image_x, center_image_y)

        depth_1 = depth_array[center_image_y, center_image_x]*0.001;

        depth_2 = depth_array[rect[1], rect[0]]*0.001;

        _intrinsics = rs2.intrinsics()
        _intrinsics.width = camera_info.width
        _intrinsics.height = camera_info.height
        _intrinsics.ppx = camera_info.k[2]
        _intrinsics.ppy = camera_info.k[5]
        _intrinsics.fx = camera_info.k[0]
        _intrinsics.fy = camera_info.k[4]
        #_intrinsics.model = cameraInfo.distortion_model
        _intrinsics.model  = rs2.distortion.brown_conrady
        _intrinsics.coeffs = [i for i in camera_info.d]  


        center_point = self.convert_pixel_to_point(center_image_x, center_image_y, depth_1, camera_info)
        corner_point = self.convert_pixel_to_point(rect[0],rect[1], depth_2, camera_info)

        color_pixel = rs2.rs2_project_point_to_pixel(_intrinsics, (center_point))
        print(color_pixel)

        #size_y = (center_point[0]-corner_point[0])*2.5
        #size_x = (center_point[1]-corner_point[1])*2.5

        #print(size_y, size_x)
        size = np.array([0.1, 0.1,0.1])

        # Define bounding box of object in Poincloud Coordinate system 
        center = np.array([center_point[0], center_point[1], center_point[2]])

        r = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        bbox = o3d.geometry.OrientedBoundingBox(center, r, size)

    

        # Visualize the bounding box and crop the pointcloud around the bounding box coordinates
        #o3d.visualization.draw_geometries([self.o3d_pcd, bbox])
        self.o3d_pcd = self.o3d_pcd.crop(bbox)
        self.o3d_pcd = self.o3d_pcd.translate((-center[0], -center[1],-0.25))

        o3d.io.write_point_cloud("copy_of_fragment.pcd", self.o3d_pcd)
        
        self.center = center
        print(self.center[0], self.center[1])

        #image = cv2.circle(self.current_frame, (color_pixel[0],color_pixel[1]), radius=0, color=(0, 0, 255), thickness=-1)
        #cv2.imshow("image", image)
        
        # self.camera_model = image_geometry.PinholeCameraModel()
        # self.camera_info = camera_info

        # Pointcloud matching with the cropped bounding box 
        # returns a transformation matrix 4x4 consisting of 
        # the rotation and translation within the cropped area
        print(self.o3d_pcd)
        if o3d.geometry.PointCloud.is_empty(self.o3d_pcd): 
            return

        self.transformation_matrix = reg.register(self.o3d_pcd)

        # transform the received pose in Point Cloud Coordinate System to 
        # World coordinates and publish it as a color pose message

        self.transform_pose(self.transformation_matrix)

    

    def convert_pixel_to_point(self,x, y, depth, cameraInfo):  
        _intrinsics = rs2.intrinsics()
        _intrinsics.width = cameraInfo.width
        _intrinsics.height = cameraInfo.height
        _intrinsics.ppx = cameraInfo.k[2]
        _intrinsics.ppy = cameraInfo.k[5]
        _intrinsics.fx = cameraInfo.k[0]
        _intrinsics.fy = cameraInfo.k[4]
        #_intrinsics.model = cameraInfo.distortion_model
        _intrinsics.model  = rs2.distortion.none
        _intrinsics.coeffs = [i for i in cameraInfo.d]  
        result = rs2.rs2_deproject_pixel_to_point(_intrinsics, [x, y], depth)
        print("Center calculation realsense")
        return [result[0], result[1], result[2]]
            



    def transform_pose(self, transformation_matrix):
        # **Assuming /tf2 topic is being broadcasted
        pose_camera_frame = tf2_geometry_msgs.PoseStamped()


        print(transformation_matrix[0][3], transformation_matrix[1][3])

        quaternion = self.matrix_to_quaternion(transformation_matrix)
        print(self.center[0], self.center[1], self.center[2])
        
        # pose = PoseStamped()
        # pose.header.stamp = self.get_clock().now().to_msg()
        # pose.header.frame_id = 'camera_color_optical_frame'
        # pose.pose.position.x = self.center[0]
        # pose.pose.position.y = self.center[1]
        # pose.pose.position.z = self.center[2]
        # pose.pose.orientation.w = 1.0

        # i = 0; 
        # while (i!=1):
        #     try:
        #         transform = self.tfBuffer.lookup_transform('world', 'camera_color_optical_frame', rcl, timeout=Duration(seconds=5.0),)
        #         transformed_pose = self.tfBuffer.transform(pose, "world")
        #        self.get_logger().info('Transformed pose: {}'.format(transformed_pose.pose))
        #         print(transform.transform.translation)
        #         print("Successful")
        #         i = 1
        #     except tf2_ros.LookupException as e:
        #         print("Non successful")
        #         self.get_logger().error('Transform lookup failed: {}'.format(str(e)))


        t = TransformStamped()
        now_stamp = self.get_clock().now()
        t.header.stamp = now_stamp.to_msg()
        t.header.frame_id = 'camera_color_optical_frame'
        t.child_frame_id = 'box_frame'
        t.transform.translation.x = self.center[0]
        t.transform.translation.y = self.center[1]
        t.transform.translation.z = self.center[2]
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0




        for i in range (0,10):
           self.tf_broadcaster.sendTransform(t)

 
    
    
    
    
    

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
