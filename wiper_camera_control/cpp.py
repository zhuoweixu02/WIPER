import math
import time
import numpy as np


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

def adjust_for_robot_radius(path, radius):
    # Increase the step size to at least the robot's diameter
    adjusted_path = []
    step_size = 2 * radius  # Minimum step size is the robot's diameter
    for (x, y) in path:
        adjusted_path.append((round(x, 2), round(y, 2)))
    return adjusted_path

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
    # Using robot radius to define margin
    margin = int(math.ceil(radius / resolution))
    # margin = 20
    # Vertical step is approximately the robot's diameter
    vertical_step = int(math.ceil(2 * radius / resolution))

    print(
        f"Applying a margin of {margin} grid units on a grid of size {width}x{height}")
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
                    # Start of a new horizontal segment
                    row_start_x = x * resolution + min(ox)
                x += moving_direction

            if row_start_x is not None:
                # End of the current segment
                row_end_x = (x - 1) * resolution + min(ox)
                # Move to the start of the segment
                path.append((row_start_x, current_y))
                # Move to the end of the segment
                path.append((row_end_x, current_y))

            moving_direction *= -1
            x = margin if moving_direction == 1 else width - margin - 1
            y += vertical_step * sweep_direction  # Increment y by the diameter of the robot
    else:
        print("No navigable path available due to margins.")

    return path  # Ensure a list is always returned


def cpp_navi(path, robot_radius, cpp_duration, cpp_frequency, cpp_steps, eraser=1):
    adjusted_path = adjust_for_robot_radius(path, robot_radius)
    for target_position in adjusted_path:
        print(f"Moving to: {target_position}")
        navigate_robot(ser, target_position, cpp_duration,
                       cpp_frequency, cpp_steps, tag_id=0, eraser=1)
        time.sleep(cpp_frequency)  # Wait before issuing the next move


def cpp(boundary_corners, eraser=1):
    chosen_quadrant = choose_quadrant()
    print(chosen_quadrant)
    # chosen_quadrant = 1
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

    cpp_navi(path, robot_radius, cpp_duration,
             cpp_frequency, cpp_steps, eraser)
