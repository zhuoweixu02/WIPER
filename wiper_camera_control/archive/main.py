import detection as dt
import wiper_camera_control.archive.report as rp
import tkinter as tk
import queue

import threading
import time

map_corners  = {}
plot_para = []
boundary_corners = []
data_queue = queue.Queue()

# Function to execute in the first thread
def thread_one():
    global map_corners, plot_para, boundary_corners
    while True:
        data_storage = dt.capture_and_process_apriltag_data(ignore_first_seconds=1, capture_duration=2)
        map_corners, plot_para, boundary_corners = dt.process_tags(data_storage)  # This will now also capture the map_corners
        # print(plot_para)
        data_queue.put((map_corners, plot_para))


# Function to execute in the second thread
def thread_two():
    global map_corners, plot_para, boundary_corners
    # while True:
    #     if (len(map_corners) > 0 and len(plot_para) > 0):
    #         print(plot_para)
    #         rp.visualize_map(map_corners, plot_para)

# Function to execute in the third thread
def thread_three():
    for _ in range(5):
        print("Thread Three")
        time.sleep(1)



if __name__ == "__main__":
    root = tk.Tk()
    app = MapVisualizer(root)

    # Start the background data collecting thread
    thread = Thread(target=data_collecting_thread, args=(app.data_queue,))
    thread.daemon = True
    thread.start()

    root.mainloop()

    # Create threads
    t1 = threading.Thread(target=thread_one)
    t2 = threading.Thread(target=thread_two)
    t3 = threading.Thread(target=thread_three)

    # Start threads
    t1.start()
    t2.start()
    t3.start()

    # Join threads (wait for them to finish)
    t1.join()
    t2.join()
    t3.join()

    print("All threads have finished executing.")


