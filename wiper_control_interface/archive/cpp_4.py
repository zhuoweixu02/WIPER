import pyrealsense2 as rs
import numpy as np
import cv2
import pupil_apriltags as at
import math  # For calculations
import serial
import time
import pandas as pd
import matplotlib.pyplot as plt


# bluetooth_init:
# connects WIPER's bluetooth, make sure you connect with the bluetooth first
# WIPER's bluetooth name: HC06, pw: 1234
def bluetooth_init():
    # bluetooth_port = '/dev/cu.HC-06'  # for Mac
    bluetooth_port = 'COM4'  # for Windows
    baud_rate = 9600
    ser = serial.Serial(bluetooth_port, baud_rate)
    time.sleep(1)  # Wait for the connection to establish
    print(f"Connected to {bluetooth_port} at {baud_rate} baud.")
    return ser

# cmd_write: 
# control WIPER
def cmd_write(ser, carx, cary, targetx, targety, mode, power):
    # carx, cary are WIPER current positions in meters
    # targetx and targety are WIPER target positions in meters
    # mode includes 1 and 2, 1 is erasers down and 2 is erasers up
    # power includes 0 and 1, 0 is power off and 1 is power on
    cmd = f"{carx:.3f},{cary:.3f}|{targetx:.3f},{targety:.3f}|{mode}|{power}\n"
    #cmd = f"{carx},{cary}|{targetx},{targety}|{mode}|{power}\n"
    print(f"Command: {cmd}")
    ser.write(cmd.encode())
    time.sleep(0.5)  # Wait for the command to be executed

# initialize_camera_and_detector:
# launches sequence to begin Intel RealSense SDK, get camera intrinsics
# launches AprilTag recognition
def initialize_camera_and_detector():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
    profile = pipeline.start(config)
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    detector = at.Detector(families='tagStandard41h12', nthreads=1, quad_decimate=1.0, quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25, debug=0)
    return pipeline, intr, detector

# rotatation_matrix_to_euler_angles:
# calculates the angles of each tags with math
def rotation_matrix_to_euler_angles(R):
    sy = math.sqrt(R[0, 0] * R[0, 0] +  R[1, 0] * R[1, 0])
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
def capture_and_process_apriltag_data(ignore_first_seconds, capture_duration):
    pipeline, intr, detector = initialize_camera_and_detector()

    data = []  # List to store tag information including relative positions and angles
    origin_id = 2  # The tag ID to be treated as the origin (0,0)
    origin_position = [0, 0, 0]  # Assuming initial origin position
    origin_yaw = 0  # Assuming initial origin yaw

    start_time = time.time()
    while time.time() - start_time < capture_duration:
        current_time = time.time()
        if current_time - start_time > ignore_first_seconds:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            tags = detector.detect(gray_image, estimate_tag_pose=True, camera_params=[intr.fx, intr.fy, intr.ppx, intr.ppy], tag_size=0.103)

            for tag in tags:
                center = np.mean(tag.corners, axis=0)
                depth = depth_frame.get_distance(int(center[0]), int(center[1])) * 1000
                X = depth * (center[0] - intr.ppx) / intr.fx
                Y = depth * (center[1] - intr.ppy) / intr.fy
                Z = depth

                if tag.pose_R is not None:
                    angles_deg = np.degrees(rotation_matrix_to_euler_angles(tag.pose_R))
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

                    data.append([tag.tag_id, relative_x, relative_y, relative_z, relative_yaw])

            cv2.imshow('AprilTag Detection', color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    pipeline.stop()
    cv2.destroyAllWindows()
    return data  # Return the data for external use

# visualize_map:
# plots out the map to verify it looks alright
def visualize_map(map_corners,plot_para):
    center_x = plot_para[0]
    center_y = plot_para[1]
    ox = plot_para[2]
    oy = plot_para[3]
    boundary_xs = plot_para[4]
    boundary_ys = plot_para[5]

    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    ax.grid(True, which='both', linestyle='--', linewidth=0.5)

    # Set axis limits dynamically based on the corners
    offset_grid = 10  # Additional space around the plotted area
    resolution = 0.1
    ax.set_xlim([min(ox) - offset_grid * resolution, max(ox) + offset_grid * resolution])
    ax.set_ylim([min(oy) - offset_grid * resolution, max(oy) + offset_grid * resolution])

    # Plot tag corners and fill exclusion zones
    for tag_id, corners in map_corners.items():
        corner_xs, corner_ys = zip(*[(corner['x'], corner['y']) for corner in corners])
        ax.plot(corner_xs + (corner_xs[0],), corner_ys + (corner_ys[0],), '-o', label=f'Tag {tag_id}')  # Close the loop for squares
        ax.fill(corner_xs, corner_ys, alpha=0.2)  # Fill exclusion zones

    ax.plot(boundary_xs, boundary_ys, 'k--', linewidth=2, label='Map Boundary')
    ax.plot(center_x, center_y, 'ro', label='Map Center')

    ax.legend(loc='upper right')
    plt.xlabel('X position (m)')
    plt.ylabel('Y position (m)')
    plt.title('Map Visualization with Exclusion Zones around Tags')
    plt.show()

# process_and_visualize_tags:
# VERY IMPORTANT FUNCTION
# gets data from capture_and_process_apriltag_data
# builds all elements needed for the map, including corners, boundaries, center, etc
def process_and_visualize_tags(data_storage):
    # Convert the list of lists to a structured array for easier processing
    df = pd.DataFrame(data_storage, columns=['Tag ID', 'X', 'Y', 'Z', 'Yaw'])
    
    # Define the tag IDs you're interested in
    tag_ids = [0, 1, 3, 4]  # Assuming these are the tag IDs you want to process
    
    averages = {2: {'X': 0.0, 'Y': 0.0}}  # Manually adding Tag 2 as origin (0, 0)
    
    for tag_id in tag_ids:
        tag_data = df[df['Tag ID'] == tag_id]
        if not tag_data.empty:
            # Convert from mm to meters directly during averaging
            avg_data = tag_data.mean() / 1000  # Convert mm to meters
            averages[tag_id] = {'X': avg_data['X'], 'Y': avg_data['Y']}

    # Define tag size with safety factor
    tag_size_with_safety = 0.103 * (1 + 0.20)  # Tag size including safety factor in meters
    half_exclusion = tag_size_with_safety / 2 
    
    # Calculating the corners for each tag
    map_corners = {}
    for tag_id, avg in averages.items():
        x, y = avg['X'], avg['Y']
        map_corners[tag_id] = [
            {'x': x - half_exclusion, 'y': y - half_exclusion},  # Bottom-left corner
            {'x': x + half_exclusion, 'y': y - half_exclusion},  # Bottom-right corner
            {'x': x + half_exclusion, 'y': y + half_exclusion},  # Top-right corner
            {'x': x - half_exclusion, 'y': y + half_exclusion},  # Top-left corner
        ]

    ox, oy = [], []
    for corners in map_corners.values():
        for corner in corners:
            ox.append(corner['x'])
            oy.append(corner['y'])
    
    for tag_id, corners in map_corners.items():
            corner_xs, corner_ys = zip(*[(corner['x'], corner['y']) for corner in corners])

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
    boundary_xs, boundary_ys = zip(*[(corner['x'], corner['y']) for corner in boundary_corners])
    boundary_xs += (boundary_xs[0],)  # Close the loop
    boundary_ys += (boundary_ys[0],)  # Close the loop

    # Calculate the center of the map
    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2
    plot_para = [center_x, center_y, ox, oy, boundary_xs, boundary_ys, corner]
    #print(map_corners)
    return map_corners, plot_para, boundary_corners

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

def get_quadrant_coordinates(boundary_corners, chosen_quadrant):
    """
    #Returns the coordinates for the four quadrants of the map based on the boundary corners.
    Args:
        boundary_corners (list): List of dictionaries containing the boundary corners.
    Returns:
        list: List of tuples containing the coordinates of each quadrant in the format (xmax, ymin), (xmin, ymin), (xmin, ymax), (xmax, ymax).
    """
    # Calculating the coordinates for the quadrants
    # quadrant1 = ((top_left[0], top_left[1]), (top_left[0], mid_y), (mid_x, mid_y), (mid_x, top_left[1]))
    # quadrant2 = ((bottom_left[0], mid_y), (bottom_left[0], bottom_left[1]), (mid_x, bottom_left[1]), (mid_x, mid_y))
    # quadrant3 = ((mid_x, mid_y), (mid_x, bottom_left[1]), (bottom_right[0], bottom_right[1]), (bottom_right[0], mid_y))
    # quadrant4 = ((mid_x, top_right[1]),(mid_x, mid_y), (top_right[0], mid_y), (top_right[0], top_right[1]))
    # Quadrant 1
    # Extracting boundary corner coordinates
    bottom_left = (boundary_corners[0]['x'], boundary_corners[0]['y'])
    bottom_right = (boundary_corners[1]['x'], boundary_corners[1]['y'])
    top_right = (boundary_corners[2]['x'], boundary_corners[2]['y'])
    top_left = (boundary_corners[3]['x'], boundary_corners[3]['y'])

    # Calculating the midpoint of the map
    mid_x = (bottom_left[0] + top_right[0]) / 2
    mid_y = (bottom_left[1] + top_right[1]) / 2

    # Quadrant 1
    q1x, q1y = [top_left[0], top_left[0], mid_x, mid_x], [top_left[1], mid_y, mid_y, top_left[1]]
    # Quadrant 2
    q2x, q2y = [bottom_left[0], bottom_left[0], mid_x, mid_x], [mid_y, bottom_left[1], bottom_left[1], mid_y]
    # Quadrant 3
    q3x, q3y = [mid_x, mid_x, bottom_right[0], bottom_right[0]], [mid_y, bottom_left[1], bottom_right[1], mid_y]
    # Quadrant 4
    q4x, q4y = [mid_x, mid_x, top_right[0], top_right[0]], [top_right[1], mid_y, mid_y, top_right[1]]
    # Quadrant 5: Entire Boar
    q5x = [bottom_left[0], bottom_left[0], top_right[0], top_right[0]]
    q5y = [bottom_left[1], top_right[1], top_right[1], bottom_left[1]]

    # Selecting the quadrant based on chosen_quadrant
    if chosen_quadrant == 1:
        return q1x, q1y
    elif chosen_quadrant == 2:
        return q2x, q2y
    elif chosen_quadrant == 3:
        return q3x, q3y
    elif chosen_quadrant == 4:
        return q4x, q4y
    elif chosen_quadrant == 5:
        return q4x, q4y
    elif chosen_quadrant == 5:
        return q5x, q5y
    else:
        raise ValueError("Invalid quadrant choice. Please choose from 1-5.")

def choose_quadrant():
    quadrants = [1,2,3,4,5]
    while True:
        try:
            choice = int(input("Please choose a quadrant (1-5), 5 is the entire whiteboard: "))
            if choice in range(1, 6):
                return quadrants[choice - 1]
            else:
                print("Invalid choice. Please choose from 1-5.")
        except ValueError:
            print("Invalid input. Please enter a number.")



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

def adjust_for_robot_radius(path, radius):
    # Increase the step size to at least the robot's diameter
    adjusted_path = []
    step_size = 2 * radius  # Minimum step size is the robot's diameter
    for (x, y) in path:
        adjusted_path.append((round(x, 2), round(y, 2)))
    return adjusted_path


def plan_path_with_radius(ox, oy, resolution, radius, moving_direction=1, sweep_direction=1):
    """
    Adjusted function to ensure vertical movements occur, taking into account grid size and step increments.
    """
    width = math.ceil((max(ox) - min(ox)) / resolution)
    height = math.ceil((max(oy) - min(oy)) / resolution)
    grid_map = np.zeros((height, width))
    margin = int(math.ceil(radius / resolution))

def plan_path_with_radius(ox, oy, resolution, radius, moving_direction=1, sweep_direction=1):
    """
    Plans a path for a robot with a specified radius using a sweep search algorithm in a grid environment.
    
    Args:
        ox (list): List of x-coordinates of grid boundaries or obstacles.
        oy (list): List of y-coordinates of grid boundaries or obstacles.
        resolution (float): The grid resolution.
        radius (float): The robot's radius.
        moving_direction (int): Initial moving direction, 1 for right, -1 for left.
        sweep_direction (int): Sweep direction, 1 for up, -1 for down.

    Returns:
        list: A list of (x, y) tuples representing the path.
    """
    width = math.ceil((max(ox) - min(ox)) / resolution)
    height = math.ceil((max(oy) - min(oy)) / resolution)
    grid_map = np.zeros((height, width))
    margin = int(math.ceil(radius / resolution))  # Using robot radius to define margin
    #margin = 20
    vertical_step = int(math.ceil(2 * radius / resolution))  # Vertical step is approximately the robot's diameter

    print(f"Applying a margin of {margin} grid units on a grid of size {width}x{height}")
    print(f"Vertical step between rows set to {vertical_step} grid units.")

    # Apply margin to grid map
    for i in range(height):
        for j in range(width):
            if i < margin or j < margin or i >= height - margin or j >= width - margin:
                grid_map[i, j] = 1

    # Check navigable space
    navigable_space = np.sum(grid_map == 0)
    print(f"Navigable space available: {navigable_space} grid cells")

    path = []
    x, y = margin, margin
    if navigable_space > 0:  # Only compute path if there's navigable space
        while y < height - margin:
            current_y = y * resolution + min(oy)
            row_start_x = None
            while x < width - margin and grid_map[y, x] == 0:
                if row_start_x is None:
                    row_start_x = x * resolution + min(ox)  # Start of a new horizontal segment
                x += moving_direction
            
            if row_start_x is not None:
                row_end_x = (x - 1) * resolution + min(ox)  # End of the current segment
                path.append((row_start_x, current_y))  # Move to the start of the segment
                path.append((row_end_x, current_y))  # Move to the end of the segment
            
            moving_direction *= -1
            x = margin if moving_direction == 1 else width - margin - 1
            y += vertical_step * sweep_direction  # Increment y by the diameter of the robot
    else:
        print("No navigable path available due to margins.")
    
    return path  # Ensure a list is always returned

    

def cpp_navi(path, robot_radius, cpp_duration, cpp_frequency, cpp_steps, eraser = 1):
    adjusted_path = adjust_for_robot_radius(path, robot_radius)
    for target_position in adjusted_path:
        print(f"Moving to: {target_position}")
        navigate_robot(ser, target_position, cpp_duration, cpp_frequency, cpp_steps, tag_id=0, eraser = 1)
        time.sleep(cpp_frequency)  # Wait before issuing the next move

def cpp(boundary_corners, eraser = 1):
    chosen_quadrant = choose_quadrant()
    print(chosen_quadrant)
    #chosen_quadrant = 1
    ox, oy = get_quadrant_coordinates(boundary_corners, chosen_quadrant)
    print(ox)
    print(oy)
 
    resolution = 0.01  # Grid resolution in meters
    robot_radius = 0.12  # Robot radius in meters
    cpp_duration = 120  # Total duration for navigation
    cpp_frequency = 3  # Position tracking and adjustment frequency
    cpp_steps = 3  # Steps to target

    path = plan_path_with_radius(ox, oy, resolution, robot_radius)
    print(path)
    
    cpp_navi(path, robot_radius, cpp_duration, cpp_frequency, cpp_steps, eraser)


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
    