import math
import time
import pyrealsense2 as rs
import numpy as np
import cv2
import pandas as pd
import pupil_apriltags as at
from scipy.stats import zscore

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
    
    # Assuming all tags are the same size and square, here's the tag size in meters
    tag_size = 0.057  # Modify this with your tag size in meters
    object_points = np.array([
        [-tag_size / 2, -tag_size / 2, 0],  # Bottom-left corner
        [tag_size / 2, -tag_size / 2, 0],  # Bottom-right corner
        [tag_size / 2, tag_size / 2, 0],  # Top-right corner
        [-tag_size / 2, tag_size / 2, 0]   # Top-left corner
    ], dtype=np.float32)

    return pipeline, detector, align, tag_size, object_points
    

def get_avg_tag(data):
    df = pd.DataFrame(data, columns=['Tag ID', 'X', 'Y', 'Z'])
    num = 1
    df['X_Z-Scores'] = zscore(df['X'])
    X_filtered_df = df[df['X_Z-Scores'].abs() <= num]
    X_mean = X_filtered_df['X'].mean()
    df['Y_Z-Scores'] = zscore(df['Y'])
    Y_filtered_df = df[df['Y_Z-Scores'].abs() <= num]
    Y_mean = Y_filtered_df['Y'].mean()
    df['Z_Z-Scores'] = zscore(df['Z'])
    Z_filtered_df = df[df['Z_Z-Scores'].abs() <= num]
    Z_mean = Z_filtered_df['Z'].mean()
    avg_tag = [df["Tag ID"][0], X_mean,
            Y_mean, Z_mean]
    return avg_tag

def draw_axis(img, corner, imgpts):
    img = cv2.line(img, corner, tuple(np.round(imgpts[0].ravel()).astype(int)), (255,0,0), 5)
    img = cv2.line(img, corner, tuple(np.round(imgpts[1].ravel()).astype(int)), (0,255,0), 5)
    img = cv2.line(img, corner, tuple(np.round(imgpts[2].ravel()).astype(int)), (0,0,255), 5)
    return img

def vec_inv(rotation_vector, translation_vector):
    # Calculate inverse transformation from the rotation and translation vectors
    R, _ = cv2.Rodrigues(rotation_vector)
    R_inv = R.T
    t_inv = -R_inv @ translation_vector.reshape(-1, 1)
    return R_inv, t_inv

# process_tags:
# VERY IMPORTANT FUNCTION
# gets data from capture_and_process_apriltag_data
# builds all elements needed for the map, including corners, boundaries, center, etc
def process_tags(data_storage):
    # Convert the list of lists to a structured array for easier processing
    df = pd.DataFrame(data_storage, columns=['Tag ID', 'X', 'Y', 'Z'])
    tag_locations = {}

    # Find the minimum and maximum values of X and Y
    tag_min_x = df['X'].min()
    tag_min_y = df['Y'].min()

    for tag_id in df['Tag ID'].unique():
        tag_data = df[df['Tag ID'] == tag_id]
        if not tag_data.empty:
            x = tag_data['X'].values[0]
            y = tag_data['Y'].values[0]
            tag_locations[tag_id] = {
                'X': x - tag_min_x, 'Y': y - tag_min_y}

    # Define tag size with safety factor
    # Tag size including safety factor in meters
    tag_size_with_safety = 0.103 * (1 + 0.20)
    half_exclusion = tag_size_with_safety / 2

    # Calculating the corners for each tag
    map_corners = {}
    # print(tag_min_x, tag_min_y, end=" ")
    for tag_id, avg in tag_locations.items():
        x, y = avg['X'], avg['Y']
        text = f"{tag_id}:({x:.2f},{y:.2f})"
        map_corners[tag_id] = [
            {'x': x - half_exclusion, 'y': y -
                half_exclusion},  # Bottom-left corner
            {'x': x + half_exclusion, 'y': y - \
                half_exclusion},  # Bottom-right corner
            {'x': x + half_exclusion, 'y': y + half_exclusion},  # Top-right corner
            {'x': x - half_exclusion, 'y': y + half_exclusion},  # Top-left corner
        ]
    #     print(text, end=", ")
    # print("")

    ox, oy = [], []
    for corners in map_corners.values():
        for corner in corners:
            ox.append(corner['x'])
            oy.append(corner['y'])

    # Additional variables for boundary definition
    min_x, min_y = float('inf'), float('inf')
    max_x, max_y = float('-inf'), float('-inf')

    # Adjust min_x, min_y, max_x, max_y based on the corners
    for tag_id, corners in map_corners.items():
        if tag_id == 0:
            continue
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
    return map_corners, plot_para, boundary_corners
