import math
import time
import pyrealsense2 as rs
import numpy as np
import cv2
import pandas as pd
import pupil_apriltags as at
from itertools import combinations

# initialize_camera_and_detector:
# launches sequence to begin Intel RealSense SDK, get camera intrinsics
# launches AprilTag recognition


def initialize_camera_and_detector():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
    pipeline.start(config)

# Setup depth sensor parameters
    align = rs.align(rs.stream.color)  # Align depth frames to color frames
    detector = at.Detector(families='tagStandard41h12', nthreads=1, quad_decimate=1.0,
                           quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25, debug=0)
    return pipeline, detector, align

# rotatation_matrix_to_euler_angles:
# calculates the angles of each tags with math


def rotation_matrix_to_euler_angles(R):
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z])

# capture_and_process_apriltag_data:
# uses the initialize_camera_and_detector function to get cam intrinsics, and
# calculates the relative positions of tags and stores them in data


def capture_and_process_apriltag_data():
    pipeline, intr, detector = initialize_camera_and_detector()

    data = []  # List to store tag information including relative positions and angles
    origin_id = 2  # The tag ID to be treated as the origin (0,0)
    origin_position = [0, 0, 0]  # Assuming initial origin position
    origin_yaw = 0  # Assuming initial origin yaw

    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(gray_image, estimate_tag_pose=True, camera_params=[
        intr.fx, intr.fy, intr.ppx, intr.ppy], tag_size=0.103)

    for tag in tags:
        center = np.mean(tag.corners, axis=0)
        depth = depth_frame.get_distance(
            int(center[0]), int(center[1])) * 1000
        X = depth * (center[0] - intr.ppx) / intr.fx
        Y = depth * (center[1] - intr.ppy) / intr.fy
        Z = depth

        if tag.pose_R is not None:
            angles_deg = np.degrees(
                rotation_matrix_to_euler_angles(tag.pose_R))
            yaw_angle = angles_deg[2]
        else:
            continue  # Skip this tag if no orientation data

        if tag.tag_id == origin_id:
            origin_position = [X, Y, Z]
            origin_yaw = yaw_angle
        else:
            relative_x = X - origin_position[0]
            relative_y = -(Y - origin_position[1])
            relative_z = Z - origin_position[2]
            relative_yaw = yaw_angle - origin_yaw

            data.append([tag.tag_id, relative_x,
                        relative_y, relative_z, relative_yaw])

    pipeline.stop()
    return data  # Return the data for external use


# process_tags:
# VERY IMPORTANT FUNCTION
# gets data from capture_and_process_apriltag_data
# builds all elements needed for the map, including corners, boundaries, center, etc
def process_tags(data_storage):
    # Convert the list of lists to a structured array for easier processing
    df = pd.DataFrame(data_storage, columns=['Tag ID', 'X', 'Y', 'Z', 'Yaw'])
    tag_locations = {}

    # Find the minimum and maximum values of X and Y
    tag_min_x = df['X'].min()
    tag_min_y = df['Y'].min()

    for tag_id in df['Tag ID'].unique():
        tag_data = df[df['Tag ID'] == tag_id]
        if not tag_data.empty:
            tag_locations[tag_id] = {
                'X': tag_data['X'].values[0] - tag_min_x, 'Y': tag_data['Y'].values[0] - tag_min_y}

    # Define tag size with safety factor
    # Tag size including safety factor in meters
    tag_size_with_safety = 0.103 * (1 + 0.20)
    half_exclusion = tag_size_with_safety / 2

    # Calculating the corners for each tag
    map_corners = {}
    for tag_id, avg in tag_locations.items():
        x, y = avg['X'], avg['Y']
        map_corners[tag_id] = [
            {'x': x - half_exclusion, 'y': y -
                half_exclusion},  # Bottom-left corner
            {'x': x + half_exclusion, 'y': y - \
                half_exclusion},  # Bottom-right corner
            {'x': x + half_exclusion, 'y': y + half_exclusion},  # Top-right corner
            {'x': x - half_exclusion, 'y': y + half_exclusion},  # Top-left corner
        ]

    ox, oy = [], []
    for corners in map_corners.values():
        for corner in corners:
            ox.append(corner['x'])
            oy.append(corner['y'])

    for tag_id, corners in map_corners.items():
        corner_xs, corner_ys = zip(
            *[(corner['x'], corner['y']) for corner in corners])

    # Additional variables for boundary definition
    min_x, min_y = float('inf'), float('inf')
    max_x, max_y = float('-inf'), float('-inf')

    # Adjust min_x, min_y, max_x, max_y based on the corners
    for tag_id, corners in map_corners.items():
        for corner in corners:
            min_x = min(min_x, corner['x'])
            min_y = min(min_y, corner['y'])
            max_x = max(max_x, corner['x'])
            max_y = max(max_y, corner['y'])

    # The bottom-left corner of Tag 2's exclusion zone already sets the bottom-left boundary.
    # Now adjust for the map boundary using the calculated min and max values.
    boundary_corners = [
        {'x': min_x, 'y': min_y},  # Bottom-left
        {'x': max_x, 'y': min_y},  # Bottom-right
        {'x': max_x, 'y': max_y},  # Top-right
        {'x': min_x, 'y': max_y},  # Top-left
    ]

    # Visualization code (continued)
    # Plotting the map boundary
    # Corrected access to dictionary elements
    boundary_xs, boundary_ys = zip(
        *[(corner['x'], corner['y']) for corner in boundary_corners])
    boundary_xs += (boundary_xs[0],)  # Close the loop
    boundary_ys += (boundary_ys[0],)  # Close the loop

    # Calculate the center of the map
    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2
    plot_para = [center_x, center_y, ox, oy, boundary_xs, boundary_ys]
    # print(map_corners)
    return map_corners, plot_para, boundary_corners
