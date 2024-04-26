import pyrealsense2 as rs
import numpy as np
import cv2
import pupil_apriltags as at
import math  # For calculations
import serial
import time
import pandas as pd


import bluetooth as bt




def track_tag_position(tag_id=0, duration_seconds=1):
    # Initialize camera and detector
    pipeline, intr, detector = initialize_camera_and_detector()
    
    origin_position = None  # To store tag 2's position
    tracked_positions = []  # To store positions and orientations relative to tag 2
    
    start_time = time.time()
    
    while time.time() - start_time < duration_seconds:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue  # Skip this iteration if frames are missing

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        tags = detector.detect(gray_image, estimate_tag_pose=True, camera_params=[intr.fx, intr.fy, intr.ppx, intr.ppy], tag_size=0.103)

        for tag in tags:
            center = np.mean(tag.corners, axis=0)
            depth = depth_frame.get_distance(int(center[0]), int(center[1])) * 1000  # Convert to mm for consistency
            X = depth * (center[0] - intr.ppx) / intr.fx / 1000
            Y = depth * (center[1] - intr.ppy) / intr.fy / 1000
            Z = depth / 1000
            if tag.pose_R is not None:
                angles_deg = np.degrees(rotation_matrix_to_euler_angles(tag.pose_R))
                yaw_angle = angles_deg[2]
            else:
                continue  # Skip this tag if no orientation data

            if tag.tag_id == 2:
                origin_position = (X, Y, Z, yaw_angle)  # Update origin position if it's tag 2

            elif tag.tag_id == tag_id and origin_position:
                # Calculate the relative position
                relative_x = X - origin_position[0]
                relative_y = -(Y - origin_position[1])
                relative_z = Z - origin_position[2]
                relative_yaw = yaw_angle - origin_position[3]
                tracked_positions.append((relative_x, relative_y, relative_z, relative_yaw))

        cv2.imshow('AprilTag Detection', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    pipeline.stop()
    cv2.destroyAllWindows()

    return tracked_positions







def navigate_robot(ser, target_position, tracking_duration, tracking_frequency, steps, tag_id=0, eraser=1):
    """
    Navigates the robot towards a target position by dynamically adjusting its path
    based on continuous tracking of its AprilTag position.
    
    Args:
        target_position (tuple): The target position (x, y) in meters.
        tracking_duration (int): Total duration to perform the navigation, in seconds.
        tracking_frequency (int): Frequency of position tracking and adjustment, in seconds.
        tag_id (int): The ID of the AprilTag attached to the robot.
    """
    start_time = time.time()
    

    while time.time() - start_time < tracking_duration:
        # Track the current position of the robot
        current_positions = track_tag_position(tag_id=tag_id, duration_seconds=3)
        if current_positions:
            # Assuming the last position in the list is the most current
            current_position = current_positions[-1][:2]  # Extract (x, y), ignoring z and yaw
            print(f"Current position: {current_position}")
            
            # Check if within 0.05m of the target position
            if math.isclose(current_position[0], target_position[0], abs_tol=0.05) and \
            math.isclose(current_position[1], target_position[1], abs_tol=0.05):
                print("Robot has reached the vicinity of the target position. Stopping navigation.")
                break

            # Calculate the next waypoint (simplified for demonstration)
            # This simplistic approach calculates a direct line towards the target; consider more sophisticated path planning as needed
            direction_x = target_position[0] - current_position[0]
            direction_y = target_position[1] - current_position[1]
            next_waypoint = (current_position[0] + direction_x / steps, current_position[1] + direction_y / steps)
            
            # Issue the command to move the robot towards the next waypoint
            # Here, mode and power are set to 1 as placeholders; adjust as needed for your application
            #ser = bluetooth_init()
            cmd_write(ser,current_position[0],current_position[1],next_waypoint[0],next_waypoint[1],eraser,1)
            # Check 
            time.sleep(1)
            if ser.inWaiting() > 0:
                response = ser.readline().decode('utf-8', 'ignore').rstrip()
                print(response)
            
            #ser.close()
        else:
            print("Failed to track the robot's position.")
        
        # Wait for the next tracking cycle
        time.sleep(tracking_frequency)

    print("Navigation completed.")



if "__main__" == __name__:
    try:
        # To dapture and print AprilTag data, ignoring the first second of data capture
        data_storage = capture_and_process_apriltag_data(ignore_first_seconds=1, capture_duration=2)

        # Visualization
        map_corners, plot_para, boundary_corners = process_and_visualize_tags(data_storage)  # This will now also capture the map_corners
        visualize_map(map_corners, plot_para)

        # Path planning: go to the center of map
        ser = bluetooth_init()
        target_position = (plot_para[0],plot_para[1]) # center of map
        tracking_duration = 60  # Navigate for 60 seconds
        tracking_frequency = 1 # Update position and adjust path every x seconds
        steps = 5 # Finish the navigation within x step

        navigate_robot(ser, target_position, tracking_duration, tracking_frequency, steps, tag_id=0, eraser = 2)


        cpp(boundary_corners, eraser = 1)

    except KeyboardInterrupt:
        pass

    ser.close()
    