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

def plan_path_with_radius(ox, oy, resolution, radius, y_offset, moving_direction=1, sweep_direction=1):
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
    vertical_step = int(math.ceil(radius / resolution))

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
            current_y = y * resolution + min(oy) + y_offset
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
            y += vertical_step * sweep_direction
    else:
        print("No navigable path available due to margins.")

    return path  # Ensure a list is always returned