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
from bluetooth import BluetoothInterface

data_storage = []
map_corners = {}
plot_para = []
boundary_corners = []
current_position = {"x": 0, "y": 0}
target_position = {"x": 0, "y": 0}
power = 0
left = 1

flag_terminate = False

tolerance = 0.05
sample_size = 100


class App:
    def __init__(self, root, bluetooth_interface):
        global power
        self.root = root
        self.bluetooth_interface = bluetooth_interface
        self.previous_messages = []
        self.previous_received_messages = []
        self.current_message_index = -1

        self.frame = tk.Frame(self.root)
        self.frame.pack(fill="both", expand=True)

        self.message_label = tk.Label(self.frame, text="Message:")
        self.message_label.grid(row=0, column=0)
        self.message_entry = tk.Entry(self.frame)
        self.message_entry.grid(row=0, column=1)

        # Bind Return key to send_message_event
        self.message_entry.bind("<Return>", self.send_message_event)

        self.send_button = tk.Button(
            self.frame, text="Send", command=self.send_message)
        self.send_button.grid(row=0, column=2)

        self.send_button = tk.Button(
            self.frame, text="Run", command=lambda: globals().update({'power': 1}))
        self.send_button.grid(row=0, column=3)

        self.send_button = tk.Button(
            self.frame, text="Stop", command=lambda: globals().update({'power': 0}))
        self.send_button.grid(row=0, column=4)

        self.send_button = tk.Button(
            self.frame, text="Exit", command=self.terminate_program)
        self.send_button.grid(row=0, column=5)

        self.message_history_label = tk.Label(
            self.frame, text="Message History:")
        self.message_history_label.grid(
            row=1, column=0, columnspan=3, sticky="w")

        self.message_history_text = tk.Text(self.frame, height=10, width=40)
        self.message_history_text.grid(row=2, column=0, columnspan=3)

        self.received_message_label = tk.Label(
            self.frame, text="Received Message:")
        self.received_message_label.grid(row=3, column=0, sticky="w")

        self.received_message_text = tk.Text(self.frame, height=10, width=160)
        self.received_message_text.grid(row=4, column=0, columnspan=3)

        # Bind up and down arrow keys to load previous messages
        self.root.bind("<Up>", self.load_previous_message)
        self.root.bind("<Down>", self.load_next_message)

        self.canvas_width = 600
        self.canvas_height = 300
        self.data_queue = queue.Queue()

        # Initialize the canvas
        self.canvas = tk.Canvas(
            root, width=self.canvas_width, height=self.canvas_height, bg='white')
        self.canvas.pack()

        # Set window title
        self.root.title("Interface")

        # Start the update checker
        self.check_for_updates()

    def terminate_program(self):
        global flag_terminate
        flag_terminate = True
        self.root.quit()

    def scale_coord(self, x, y, minx, miny, maxy):
        """ Scale map coordinates to fit the canvas """
        scale = 200  # Adjust scale factor as needed
        offset = 10
        return ((x - minx) * scale + offset, (self.canvas_height - (y - miny) * scale - (self.canvas_height - (maxy - miny) * scale)) + offset)

    def draw_map(self, map_corners, plot_para):
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
        maxy = max([y for x, y in zip(boundary_xs, boundary_ys)])

        # Draw boundary
        scaled_boundary_coords = [self.scale_coord(
            x, y, minx, miny, maxy) for x, y in zip(boundary_xs, boundary_ys)]
        self.canvas.create_polygon(
            *scaled_boundary_coords, outline='black', fill='', dash=(4, 2))

        # Draw tags and exclusion zones
        for tag_id, corners in map_corners.items():
            scaled_corners = [self.scale_coord(
                corner['x'], corner['y'], minx, miny, maxy) for corner in corners]
            xs, ys = zip(*scaled_corners)
            self.canvas.create_polygon(
                *scaled_corners, outline='blue', fill='lightblue', tags=f'Tag {tag_id}')
            self.canvas.create_text(
                sum(xs)/len(xs), sum(ys)/len(ys), text=f'Tag {tag_id}')

        # Draw the center point
        center_coords = self.scale_coord(center_x, center_y, minx, miny, maxy)
        self.canvas.create_oval(
            center_coords[0]-5, center_coords[1]-5, center_coords[0]+5, center_coords[1]+5, fill='red')

    def check_for_updates(self):
        """ Check the queue for new data and update the map if necessary """
        try:
            map_corners, plot_para = self.data_queue.get_nowait()
            self.draw_map(map_corners, plot_para)
        except queue.Empty:
            pass
        finally:
            self.root.after(1, self.check_for_updates)

    def send_message(self):
        message = self.message_entry.get()
        self.bluetooth_interface.send_message(message)
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
    global map_corners, plot_para, boundary_corners, current_position, data_storage, flag_terminate
    pipeline, detector, align = dt.initialize_camera_and_detector()
    while not flag_terminate:
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
        tag_info = []
        data_storage = []

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

            # Get depth and calculate real-world coordinates
            depth = depth_frame.get_distance(ptCenter[0], ptCenter[1])
            depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            realworld_coords = rs.rs2_deproject_pixel_to_point(
                depth_intrinsics, [ptCenter[0], ptCenter[1]], depth)
            correct_coords = [realworld_coords[0], -realworld_coords[1], realworld_coords[2]]
            tag_info.append((ptCenter, correct_coords, r.tag_id))
            one_tag = [r.tag_id, correct_coords[0],
                       correct_coords[1], correct_coords[2], 0]
            data_storage.append(one_tag)



            # Annotate the tag ID and its real-world coordinates
            cv2.putText(color_image, f"ID: {r.tag_id} XYZ: {np.round(correct_coords, 2)}m",
                        (ptA[0], ptA[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

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
        data_queue.put((map_corners, plot_para))
    pipeline.stop()
    cv2.destroyAllWindows()


def cmd_write_thread(bluetooth_interface):
    global power, left, flag_terminate, current_position, target_position
    while not flag_terminate:
        cmd = f"{current_position['x']:.3f},{current_position['y']:.3f}|{target_position['x']:.3f},{target_position['y']:.3f}|{left}|{power}\n"
        bluetooth_interface.send_message(cmd)
        time.sleep(0.5)


def navigation_thread():
    global current_position, target_position, plot_para, flag_terminate, power
    while not flag_terminate:
        try:
            target_position = {"x": plot_para[0], "y": plot_para[1]}
            if (abs(current_position['x'] - target_position['x']) < tolerance) and (abs(current_position['y'] - target_position['y']) < tolerance):
                power = 0
        except:
            pass
        time.sleep(0.1)


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

    # Start the background data collecting thread
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
