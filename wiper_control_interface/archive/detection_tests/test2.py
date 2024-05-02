import pyrealsense2 as rs
import cv2
import pupil_apriltags as apriltag
import numpy as np

# Initialize RealSense camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# Setup depth sensor parameters
align = rs.align(rs.stream.color)  # Align depth frames to color frames

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

        # Loop over the detected AprilTags
        for r in results:
            # Extract the bounding box and centroid
            (ptA, ptB, ptC, ptD) = r.corners
            ptA = np.round(ptA).astype("int")
            ptB = np.round(ptB).astype("int")
            ptC = np.round(ptC).astype("int")
            ptD = np.round(ptD).astype("int")
            ptCenter = (int(r.center[0]), int(r.center[1]))

            # Draw the bounding box
            cv2.line(color_image, tuple(ptA), tuple(ptB), (0, 255, 0), 2)
            cv2.line(color_image, tuple(ptB), tuple(ptC), (0, 255, 0), 2)
            cv2.line(color_image, tuple(ptC), tuple(ptD), (0, 255, 0), 2)
            cv2.line(color_image, tuple(ptD), tuple(ptA), (0, 255, 0), 2)

            # Draw the centroid of the tag
            cv2.circle(color_image, ptCenter, 5, (0, 0, 255), -1)

            # Annotate the tag ID and its real-world coordinates
            depth = depth_frame.get_distance(ptCenter[0], ptCenter[1])
            depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            realworld_coords = rs.rs2_deproject_pixel_to_point(
                depth_intrinsics, [ptCenter[0], ptCenter[1]], depth)
            cv2.putText(color_image, f"ID: {r.tag_id} XYZ: {np.round(realworld_coords, 2)}m",
                        (ptA[0], ptA[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        # Display the resulting frame
        cv2.imshow('Frame', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop the pipeline
    pipeline.stop()
    cv2.destroyAllWindows()
