import tkinter as tk
import threading
import atexit
import queue
import serial

import detection as dt


class BluetoothInterface:
    def __init__(self, port, baudrate, app):
        self.port = port
        self.baudrate = baudrate
        self.serial_port = serial.Serial(
            port=self.port, baudrate=self.baudrate, timeout=1)
        self.app = app  # Reference to the App instance
        # Event to signal the thread to stop
        self.receive_thread_stop = threading.Event()
        self.receive_thread = threading.Thread(target=self.receive_data)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        # Register close_serial to be called when the program exits
        atexit.register(self.close_serial)

    def send_message(self, message):
        self.serial_port.write(message.encode())

    def receive_data(self):
        while not self.receive_thread_stop.is_set():
            if self.serial_port.in_waiting > 0:
                try:
                    received_data = self.serial_port.readline().decode().strip()
                except UnicodeDecodeError:
                    pass
                # Update GUI with received message
                self.app.master.after(
                    0, self.app.update_received_message, received_data)
                # print("WIPER:", received_data)
            # time.sleep(0.1)  # Add a small delay to avoid busy waiting


class App:
    def __init__(self, root, bluetooth_interface):
        self.root = root
        self.bluetooth_interface = bluetooth_interface
        self.previous_messages = []
        self.previous_received_messages = []
        self.current_message_index = -1

        self.frame = tk.Frame(self.root)
        self.frame.pack()

        self.message_label = tk.Label(self.frame, text="Message:")
        self.message_label.grid(row=0, column=0)

        self.canvas_width = 600
        self.canvas_height = 400
        self.data_queue = queue.Queue()

        # Initialize the canvas
        self.canvas = tk.Canvas(
            root, width=self.canvas_width, height=self.canvas_height, bg='white')
        self.canvas.pack()

        # Set window title
        self.root.title("Interface")

        # Start the update checker
        self.check_for_updates()

    def scale_coord(self, x, y):
        """ Scale map coordinates to fit the canvas """
        scale = 200  # Adjust scale factor as needed
        return (x * scale + 100, self.canvas_height - (y * scale + 200))

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

        # Draw boundary
        scaled_boundary_coords = [self.scale_coord(
            x, y) for x, y in zip(boundary_xs, boundary_ys)]
        self.canvas.create_polygon(
            *scaled_boundary_coords, outline='black', fill='', dash=(4, 2))

        # Draw tags and exclusion zones
        for tag_id, corners in map_corners.items():
            scaled_corners = [self.scale_coord(
                corner['x'], corner['y']) for corner in corners]
            xs, ys = zip(*scaled_corners)
            self.canvas.create_polygon(
                *scaled_corners, outline='blue', fill='lightblue', tags=f'Tag {tag_id}')
            self.canvas.create_text(
                sum(xs)/len(xs), sum(ys)/len(ys), text=f'Tag {tag_id}')

        # Draw the center point
        center_coords = self.scale_coord(center_x, center_y)
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
            self.root.after(100, self.check_for_updates)


def data_collecting_thread(data_queue):
    # Simulate changing data
    while True:
        data_storage = dt.capture_and_process_apriltag_data(
            ignore_first_seconds=0, capture_duration=1)
        map_corners, plot_para, boundary_corners = dt.process_tags(
            data_storage)  # This will now also capture the map_corners
        data_queue.put((map_corners, plot_para))


def receive_message_thread(ser):
    while True:


if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)

    # Start the background data collecting thread
    thread = threading.Thread(
        target=data_collecting_thread, args=(app.data_queue,))
    thread.daemon = True
    thread.start()

    root.mainloop()
