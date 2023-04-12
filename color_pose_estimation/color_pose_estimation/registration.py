##################################################
# Registration of point clouds
##################################################
# Copyright 2018 - 2021, www.open3d.org
##################################################
# http://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html,
##################################################

import numpy as np
import copy
import json
import argparse
import time
import matplotlib.pyplot as plt
import open3d as o3d
import numpy as np
import sys
import cv2 as cv2
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down=pcd
    pcd_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


def prepare_dataset(target, voxel_size):
    print(":: Load two point clouds")
    #o3d.visualization.draw_geometries([target])

    path = Path(get_package_share_directory("color_pose_estimation")).joinpath("cube2.ply")
    print(path)

    source = o3d.io.read_point_cloud(str(path))

    print(type(source))
    print(type(target))
    #bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(0, 0, 0), max_bound=(10, 30, 10))
    #target = target.crop(bbox)

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()

    #o3d.visualization.draw_geometries([target_down, mesh])
    #o3d.visualization.draw_geometries([source_down, mesh])

    return source, target, source_down, target_down, source_fpfh, target_fpfh


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 50
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.2),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(10000, 0.999))
    return result


def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size, result_ransac):
    distance_threshold = voxel_size * 30
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(), 
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100))
    return result


def register(target):
    voxel_size = 0.05  # means 40 cm for this dataset

    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(target,
        voxel_size)
    result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)
    print(result_ransac)
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()

    #draw_registration_result(source_down, target_down,
    #                         result_ransac.transformation)
    result_icp = refine_registration(
        source, target, source_fpfh, target_fpfh, voxel_size, result_ransac)
    print(result_icp)
    # Transformation value can be used to calculate pose
    print(result_icp.transformation)
    #draw_registration_result(source, target, result_icp.transformation)
    return result_icp.transformation


