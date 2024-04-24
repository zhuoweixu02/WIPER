import tkinter as tk
import threading

import detection as dt
from bluetooth import BluetoothInterface
from interface import App


def data_collecting_thread(data_queue):
    # Simulate changing data
    while True:
        data_storage = dt.capture_and_process_apriltag_data(
            ignore_first_seconds=0, capture_duration=1)
        map_corners, plot_para, boundary_corners = dt.process_tags(
            data_storage)  # This will now also capture the map_corners
        data_queue.put((map_corners, plot_para))


if __name__ == "__main__":
    bluetooth_port = '/dev/cu.HC-06'  # for Mac
    # bluetooth_port = 'COM6'  # for Windows

    root = tk.Tk()
    root.title("WIPER CONTROL")
    app = App(root, None)  # Pass None initially for the BluetoothInterface

    bluetooth_interface = BluetoothInterface(
        port=bluetooth_port, baudrate=9600, app=app)
    # Update app's BluetoothInterface reference
    app.bluetooth_interface = bluetooth_interface

    # Start the background data collecting thread
    thread = threading.Thread(
        target=data_collecting_thread, args=(app.data_queue,))
    thread.daemon = True
    thread.start()

    root.mainloop()
