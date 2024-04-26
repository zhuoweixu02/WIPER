import pyrealsense2 as rs
import pupil_apriltags as at
import cv2
import numpy as np
import tkinter as tk
import threading
import queue
import time
from itertools import combinations

import detection as dt
import cpp
from bluetooth import BluetoothInterface


origin_id = 2
boundary_corners = []
path = []
quadrant = 0
current_position = {"x": 0, "y": 0}
target_position = {"x": 0, "y": 0}
power = 0
mode = 2

flag_terminate = False
flag_detectionReady = False

tolerance = 0.05
resolution = 0.01  # Grid resolution in meters
robot_radius = 0.12  # Robot radius in meters
sample_size = 5


class App:
    def __init__(self, root, bluetooth_interface):
        global power, quadrant
        self.root = root
        self.bluetooth_interface = bluetooth_interface
        self.previous_messages = []
        self.previous_received_messages = []
        self.current_message_index = -1
        self.data_queue = queue.Queue()

        self.upper_frame = tk.Frame(
            self.root, pady=20)
        self.upper_frame.grid(row=0, column=0)

        self.lower_frame = tk.Frame(self.root, padx=20, pady=20)
        self.lower_frame.grid(row=1, column=0)

        self.upper_frame_left = tk.Frame(
            self.upper_frame, padx=20)
        self.upper_frame_left.grid(row=0, column=0)

        self.upper_frame_right = tk.Frame(
            self.upper_frame, padx=20)
        self.upper_frame_right.grid(row=0, column=3)

        self.button_size = 5

        self.quadrant1 = tk.Button(
            self.upper_frame_right, text="1", width=self.button_size, height=self.button_size, command=lambda: globals().update({'quadrant': 1}))
        self.quadrant1.grid(row=0, column=0)

        self.quadrant2 = tk.Button(
            self.upper_frame_right, text="2", width=self.button_size, height=self.button_size, command=lambda: globals().update({'quadrant': 2}))
        self.quadrant2.grid(row=1, column=0)

        self.quadrant3 = tk.Button(
            self.upper_frame_right, text="3", width=self.button_size, height=self.button_size, command=lambda: globals().update({'quadrant': 3}))
        self.quadrant3.grid(row=1, column=1)

        self.quadrant4 = tk.Button(
            self.upper_frame_right, text="4", width=self.button_size, height=self.button_size, command=lambda: globals().update({'quadrant': 4}))
        self.quadrant4.grid(row=0, column=1)

        self.reset = tk.Button(
            self.upper_frame_right, text="Reset", width=self.button_size, command=lambda: globals().update({'quadrant': -1}))
        self.reset.grid(row=2, column=0, columnspan=2)

        self.up = tk.Button(
            self.upper_frame_right, text="UP", width=self.button_size, command=lambda: globals().update({'mode': 2}))
        self.up.grid(row=3, column=0, pady=10)

        self.down = tk.Button(
            self.upper_frame_right, text="DOWN", width=self.button_size, command=lambda: globals().update({'mode': 1}))
        self.down.grid(row=3, column=1, pady=10)

        self.out = tk.Button(
            self.upper_frame_right, text="OUT", width=self.button_size, command=lambda: globals().update({'mode': 3}))
        self.out.grid(row=4, column=0, columnspan=2)

        self.canvas_width = 600
        self.canvas_height = 300
        self.canvas = tk.Canvas(
            self.upper_frame, width=self.canvas_width, height=self.canvas_height, border=2, relief="solid")
        self.canvas.grid(row=0, column=1, padx=10)

        self.message_label = tk.Label(self.upper_frame_left, text="Message:")
        self.message_label.grid(row=0, column=0)

        self.message_entry = tk.Entry(self.upper_frame_left)
        self.message_entry.grid(row=0, column=1)

        self.send_button = tk.Button(
            self.upper_frame_left, text="Send", command=self.send_message)
        self.send_button.grid(row=0, column=2)

        self.send_button = tk.Button(
            self.upper_frame_left, text="Run", command=lambda: globals().update({'power': 1}))
        self.send_button.grid(row=0, column=3)

        self.send_button = tk.Button(
            self.upper_frame_left, text="Stop", command=lambda: globals().update({'power': 0}))
        self.send_button.grid(row=0, column=4)

        self.send_button = tk.Button(
            self.upper_frame_left, text="Exit", command=self.terminate_program)
        self.send_button.grid(row=0, column=5)

        self.message_history_label = tk.Label(
            self.upper_frame_left, text="History:")
        self.message_history_label.grid(
            row=1, column=0, sticky="w")

        self.message_history_text = tk.Text(
            self.upper_frame_left, height=10, width=50)
        self.message_history_text.grid(row=2, column=0, columnspan=6)

        self.status_label = tk.Label(self.upper_frame_left, text="")
        self.status_label.grid(row=3, column=0, columnspan=6)

        self.received_message_label = tk.Label(
            self.lower_frame, text="Received Message:")
        self.received_message_label.grid(
            row=0, column=0, sticky="w")

        self.received_message_text = tk.Text(
            self.lower_frame, height=10, width=160)
        self.received_message_text.grid(row=1, column=0)

        # Bind up and down arrow keys to load previous messages
        self.root.bind("<Up>", self.load_previous_message)
        self.root.bind("<Down>", self.load_next_message)
        self.message_entry.bind("<Return>", self.send_message_event)

        # Set window title
        self.root.title("WIPER Control Interface")
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)

        # Start the update checker
        self.check_for_updates()

    def terminate_program(self):
        global flag_terminate
        flag_terminate = True
        self.root.quit()

    def scale_coord(self, x, y, minx, maxx, miny, maxy):
        """ Scale map coordinates to fit the canvas """
        scale = 200  # Adjust scale factor as needed
        offsetx = (self.canvas_width - (maxx - minx) * scale)/2
        offsety = (self.canvas_height - ((maxy - miny) * scale))/2
        return ((x - minx) * scale + offsetx, (self.canvas_height - (y - miny) * scale - (self.canvas_height - (maxy - miny) * scale)) + offsety)

    def draw_map(self, map_corners, plot_para, path):
        """ Draw the boundary, tags, and center point on the canvas """
        center_x = plot_para[0]
        center_y = plot_para[1]
        ox = plot_para[2]
        oy = plot_para[3]
        boundary_xs = plot_para[4]
        boundary_ys = plot_para[5]

        # Clear the previous map
        self.canvas.delete("all")

        minx = min([x for x, y in zip(boundary_xs, boundary_ys)])
        miny = min([y for x, y in zip(boundary_xs, boundary_ys)])
        maxx = max([x for x, y in zip(boundary_xs, boundary_ys)])
        maxy = max([y for x, y in zip(boundary_xs, boundary_ys)])

        # Draw boundary
        scaled_boundary_coords = [self.scale_coord(
            x, y, minx, maxx, miny, maxy) for x, y in zip(boundary_xs, boundary_ys)]
        self.canvas.create_polygon(
            *scaled_boundary_coords, outline='black', fill='', dash=(4, 2))

        # Draw tags and exclusion zones
        for tag_id, corners in map_corners.items():
            scaled_corners = [self.scale_coord(
                corner['x'], corner['y'], minx, maxx, miny, maxy) for corner in corners]
            xs, ys = zip(*scaled_corners)
            self.canvas.create_polygon(
                *scaled_corners, outline='blue', fill='lightblue', tags=f'Tag {tag_id}')
            self.canvas.create_text(
                sum(xs)/len(xs), sum(ys)/len(ys), text=f'Tag {tag_id}')

        for i in range(0, len(path)):
            point1x, point1y = self.scale_coord(
                path[i][0], path[i][1], minx, maxx, miny, maxy)
            if (i < len(path) - 1):
                point2x, point2y = self.scale_coord(
                    path[i+1][0], path[i+1][1], minx, maxx, miny, maxy)
                self.canvas.create_line(
                    point1x, point1y, point2x, point2y, fill='lightblue', width=3)
                if (i == 0):
                    self.canvas.create_oval(
                        point1x-5, point1y-5, point1x+5, point1y+5, fill='green')
            else:
                self.canvas.create_oval(
                    point1x-5, point1y-5, point1x+5, point1y+5, fill='blue')

        # Draw the center point
        center_coords = self.scale_coord(
            center_x, center_y, minx, maxx, miny, maxy)
        self.canvas.create_oval(
            center_coords[0]-5, center_coords[1]-5, center_coords[0]+5, center_coords[1]+5, fill='red')

    def check_for_updates(self):
        """ Check the queue for new data and update the map if necessary """
        try:
            map_corners, plot_para, path, status = self.data_queue.get_nowait()
            self.status_label.config(
                text=f"Power: {status[0]}\nMode: {status[1]}\nQuadrant: {status[2]}\nCurrent Position: ({status[3]['x']:.2f}, {status[3]['y']:.2f})\nTarget Position: \t({status[4]['x']:.2f}, {status[4]['y']:.2f})")
            self.draw_map(map_corners, plot_para, path)
        except queue.Empty:
            pass
        finally:
            self.root.after(1, self.check_for_updates)

    def send_message(self):
        global power, mode, target_position
        message = self.message_entry.get()
        # self.bluetooth_interface.send_message(message)
        try:
            target_position['x'], target_position['y'] = map(
                float, message.split("|")[1].split(","))
            mode = int(message.split("|")[2])
            power = int(message.split("|")[3])
        except:
            pass
        self.previous_messages.insert(0, message)
        self.current_message_index = -1
        self.message_entry.delete(0, tk.END)
        self.update_message_history()

    def send_message_event(self, event):
        self.send_message()

    def load_previous_message(self, event):
        if self.current_message_index < len(self.previous_messages) - 1:
            self.current_message_index += 1
        self.update_message_entry()

    def load_next_message(self, event):
        if self.current_message_index > 0:
            self.current_message_index -= 1
        elif self.current_message_index == 0:
            self.current_message_index = -1
        self.update_message_entry()

    def update_message_entry(self):
        if self.current_message_index == -1:
            self.message_entry.delete(0, tk.END)
        else:
            self.message_entry.delete(0, tk.END)
            self.message_entry.insert(
                0, self.previous_messages[self.current_message_index])

    def update_message_history(self):
        self.message_history_text.delete("1.0", tk.END)
        for message in reversed(self.previous_messages):
            self.message_history_text.insert(tk.END, message + "\n")

    def update_received_message(self, received_message):
        self.previous_received_messages.insert(0, received_message)
        if (len(self.previous_received_messages) > 10):
            self.previous_received_messages.pop()
        self.received_message_text.delete("1.0", tk.END)
        for message in reversed(self.previous_received_messages):
            self.received_message_text.insert(tk.END, message + "\n")


def data_collecting_thread(data_queue):
    # Simulate changing data
    global sample_size, path, boundary_corners, current_position, flag_terminate, origin_id, flag_detectionReady
    flag_hasReference = False
    origin_rvec = []
    origin_tvec = []
    data_storage = []
    map_corners = {}
    plot_para = []
    samples = {}

    pipeline, detector, align, tag_size, object_points = dt.initialize_camera_and_detector()

    while not flag_terminate:
        tag_info = []
        data_storage = []

        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
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
        # Assuming no distortion
        dist_coeffs = np.zeros((4, 1), dtype=np.float32)

        # Loop over the detected AprilTags
        for r in results:
            # Extract the bounding box and centroid
            (ptA, ptB, ptC, ptD) = r.corners
            ptCenter = (int(r.center[0]), int(r.center[1]))

            # Image points must be in the same order as object points
            image_points = np.array([ptA, ptB, ptC, ptD], dtype=np.float32)

            # Solve PnP
            axis = np.float32([[tag_size, 0, 0], [0, tag_size, 0], [
                              0, 0, tag_size]]).reshape(-1, 3)
            success, rotation_vector, translation_vector = cv2.solvePnP(
                object_points, image_points, camera_matrix, dist_coeffs)
            imgpts, jac = cv2.projectPoints(
                axis, rotation_vector, translation_vector, camera_matrix, dist_coeffs)
            color_image = dt.draw_axis(color_image, ptCenter, imgpts)

            if (r.tag_id == origin_id):
                origin_rvec = rotation_vector
                origin_tvec = translation_vector
                flag_hasReference = True

            ptA = np.round(ptA).astype("int")
            ptB = np.round(ptB).astype("int")
            ptC = np.round(ptC).astype("int")
            ptD = np.round(ptD).astype("int")

            # Draw the bounding box
            cv2.line(color_image, tuple(ptA), tuple(ptB), (0, 255, 0), 2)
            cv2.line(color_image, tuple(ptB), tuple(ptC), (0, 255, 0), 2)
            cv2.line(color_image, tuple(ptC), tuple(ptD), (0, 255, 0), 2)
            cv2.line(color_image, tuple(ptD), tuple(ptA), (0, 255, 0), 2)

            # Get depth and calculate real-world coordinates
            depth = depth_frame.get_distance(ptCenter[0], ptCenter[1])
            point_in_camera_space = rs.rs2_deproject_pixel_to_point(
                intrinsics, [ptCenter[0], ptCenter[1]], depth)
            R_inv, t_inv = dt.vec_inv(rotation_vector, translation_vector)
            if flag_hasReference:
                R_inv, t_inv = dt.vec_inv(origin_rvec, origin_tvec)
            point_in_reference_space = R_inv @ np.array(
                point_in_camera_space) + t_inv.flatten()

            realworld_coords = point_in_reference_space

            # Annotate the tag ID and its real-world coordinates
            cv2.putText(color_image, f"{r.tag_id}: {np.round(realworld_coords, 2)}m",
                        (ptA[0] - 80, ptA[1] - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

            tag_info.append((ptCenter, realworld_coords, r.tag_id))
            one_tag = [r.tag_id, realworld_coords[0],
                       realworld_coords[1], realworld_coords[2]]
            if (r.tag_id not in samples.keys()):
                samples[r.tag_id] = []
            samples[r.tag_id].append(one_tag)
            if (len(samples[r.tag_id]) > sample_size):
                avg_tag = dt.get_avg_tag(samples[r.tag_id])
                data_storage.append(avg_tag)
                samples[r.tag_id].pop(0)
        #         text = f"{avg_tag[0]}:({avg_tag[1]:.2f},{avg_tag[2]:.2f})"
        #         print(text, end=", ")
        # print("")

        # Calculate real-world distances between each pair of tags and draw lines
        for (pt1, coords1, id1), (pt2, coords2, id2) in combinations(tag_info, 2):
            cv2.line(color_image, pt1, pt2, (255, 0, 0), 2)
            distance = np.linalg.norm(np.array(coords1) - np.array(coords2))
            midpoint = ((pt1[0] + pt2[0]) // 2, (pt1[1] + pt2[1]) // 2)
            cv2.putText(color_image, f"{distance:.2f}m", midpoint,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # Display the resulting frame
        cv2.imshow('Frame', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        map_corners, plot_para, boundary_corners = dt.process_tags(
            data_storage)  # This will now also capture the map_corners
        try:
            x = sum([corner['x'] for corner in map_corners[0]])/4
            y = sum([corner['y'] for corner in map_corners[0]])/4
            current_position = {"x": x, "y": y}
        except:
            pass
        status = [power, mode, quadrant, current_position, target_position]
        data_queue.put((map_corners, plot_para, path, status))

        if (len(data_storage) >= 5):
            flag_detectionReady = True
        else:
            flag_detectionReady = False

        # time.sleep(0.01)
    pipeline.stop()
    cv2.destroyAllWindows()


def cmd_write_thread(bluetooth_interface):
    global power, mode, flag_terminate, current_position, target_position
    while not flag_terminate:
        cmd = f"{current_position['x']:.3f},{current_position['y']:.3f}|{target_position['x']:.3f},{target_position['y']:.3f}|{mode}|{power}\n"
        bluetooth_interface.send_message(cmd)
        time.sleep(0.5)


def navigation_thread():
    global mode, path, quadrant, current_position, target_position, flag_terminate, power, flag_detectionReady, boundary_corners, robot_radius, resolution
    flag_pathGenerated = False
    while not flag_terminate:
        if (abs(current_position['x'] - target_position['x']) < tolerance) and (abs(current_position['y'] - target_position['y']) < tolerance):
            power = 0
        if quadrant == -1:
            quadrant = 0
            power = 0
            mode = 2
            flag_pathGenerated = False
            path = []
        if flag_detectionReady and quadrant in [1, 2, 3, 4]:
            if not flag_pathGenerated:
                ox, oy = cpp.get_quadrant_coordinates(
                    boundary_corners, quadrant)
                path = cpp.plan_path_with_radius(
                    ox, oy, resolution, robot_radius)
                if (len(path) > 0):
                    flag_pathGenerated = True
                    target_position = {"x": path[0][0], "y": path[0][1]}
                    mode = 2
            elif (len(path) > 0):
                if (abs(current_position['x'] - target_position['x']) < tolerance) and (abs(current_position['y'] - target_position['y']) < tolerance):
                    path.pop(0)
                    mode = 1
                    power = 1
                target_position = {"x": path[0][0], "y": path[0][1]}
            else:
                quadrant = -1

        time.sleep(0.01)


if __name__ == "__main__":
    # bluetooth_port = '/dev/cu.HC-06'  # for Mac
    bluetooth_port = 'COM4'  # for Windows

    root = tk.Tk()
    root.title("WIPER CONTROL")
    app = App(root, None)  # Pass None initially for the BluetoothInterface

    # bluetooth_interface = BluetoothInterface(
    #     port=bluetooth_port, baudrate=9600, app=app)
    # # Update app's BluetoothInterface reference
    # app.bluetooth_interface = bluetooth_interface

    thread1 = threading.Thread(
        target=data_collecting_thread, args=(app.data_queue,))
    thread1.daemon = True
    thread1.start()

    # thread2 = threading.Thread(
    #     target=cmd_write_thread, args=(bluetooth_interface,))
    # thread2.daemon = True
    # thread2.start()

    thread3 = threading.Thread(
        target=navigation_thread)
    thread3.daemon = True
    thread3.start()

    root.mainloop()
