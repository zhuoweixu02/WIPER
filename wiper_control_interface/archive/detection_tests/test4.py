import pyrealsense2 as rs
import cv2
import pupil_apriltags as apriltag
import numpy as np
from itertools import combinations

# Initialize RealSense camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# Setup depth sensor parameters
align = rs.align(rs.stream.color)  # Align depth frames to color frames

# Assuming all tags are the same size and square, here's the tag size in meters
tag_size = 0.057  # Modify this with your tag size in meters
object_points = np.array([
    [-tag_size / 2, -tag_size / 2, 0],  # Bottom-left corner
    [tag_size / 2, -tag_size / 2, 0],  # Bottom-right corner
    [tag_size / 2, tag_size / 2, 0],  # Top-right corner
    [-tag_size / 2, tag_size / 2, 0]   # Top-left corner
], dtype=np.float32)

def draw_axis(img, corner, imgpts):
    img = cv2.line(img, corner, tuple(np.round(imgpts[0].ravel()).astype(int)), (255,0,0), 5)
    img = cv2.line(img, corner, tuple(np.round(imgpts[1].ravel()).astype(int)), (0,255,0), 5)
    img = cv2.line(img, corner, tuple(np.round(imgpts[2].ravel()).astype(int)), (0,0,255), 5)
    return img

try:
    # Create AprilTag detector
    detector = apriltag.Detector(families='tagStandard41h12', nthreads=1, quad_decimate=1.0,
                                 quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25, debug=0)

    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Convert BGR to grayscale
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags in the grayscale image
        results = detector.detect(gray_image)

        # Camera intrinsics
        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        camera_matrix = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]
        ], dtype=np.float32)
        dist_coeffs = np.zeros((4, 1), dtype=np.float32)  # Assuming no distortion

        tag_info = []
        
        # Loop over the detected AprilTags
        for r in results:
            # Extract the bounding box and centroid
            (ptA, ptB, ptC, ptD) = r.corners

            ptCenter = (int(r.center[0]), int(r.center[1]))

            # Image points must be in the same order as object points
            image_points = np.array([ptA, ptB, ptC, ptD], dtype=np.float32)

            # Solve PnP
            axis = np.float32([[tag_size,0,0], [0,tag_size,0], [0,0,tag_size]]).reshape(-1,3)
            success, rotation_vector, translation_vector = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
            imgpts, jac = cv2.projectPoints(axis, rotation_vector, translation_vector, camera_matrix, dist_coeffs)
            color_image = draw_axis(color_image,ptCenter,imgpts)

            
            # Draw the bounding box
            ptA = np.round(ptA).astype("int")
            ptB = np.round(ptB).astype("int")
            ptC = np.round(ptC).astype("int")
            ptD = np.round(ptD).astype("int")
            cv2.line(color_image, tuple(ptA), tuple(ptB), (0, 255, 0), 2)
            cv2.line(color_image, tuple(ptB), tuple(ptC), (0, 255, 0), 2)
            cv2.line(color_image, tuple(ptC), tuple(ptD), (0, 255, 0), 2)
            cv2.line(color_image, tuple(ptD), tuple(ptA), (0, 255, 0), 2)

            # Get depth and calculate real-world coordinates
            depth = depth_frame.get_distance(ptCenter[0], ptCenter[1])
            depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            realworld_coords = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [ptCenter[0], ptCenter[1]], depth)
            tag_info.append((ptCenter, realworld_coords, r.tag_id, rotation_vector, translation_vector))

            # Annotate the tag ID and its real-world coordinates
            cv2.putText(color_image, f"ID: {r.tag_id} XYZ: {np.round(realworld_coords, 2)}m", 
                        (ptA[0], ptA[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            
            # Assuming you have pixel coordinates (u, v)
            depth = depth_frame.get_distance(ptA[0], ptA[1])

            # Deproject pixel to point in camera space
            point_in_camera_space = rs.rs2_deproject_pixel_to_point(
                intrinsics, [ptA[0], ptA[1]], depth
)
            # Calculate inverse transformation from the rotation and translation vectors
            R, _ = cv2.Rodrigues(rotation_vector)
            R_inv = R.T
            t_inv = -R_inv @ translation_vector.reshape(-1, 1)

            # Transform the point to world coordinates
            point_in_world_space = R_inv @ np.array(point_in_camera_space) + t_inv.flatten()
            print(rotation_vector)
            cv2.circle(color_image, tuple(ptA), 10, (0, 0, 255))

        # Display the resulting frame
        cv2.imshow('Frame', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop the pipeline
    pipeline.stop()
    cv2.destroyAllWindows()
