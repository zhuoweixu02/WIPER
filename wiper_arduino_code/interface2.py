import serial
import threading
import tkinter as tk
import atexit
import time


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
            # time.sleep(1)  # Add a small delay to avoid busy waiting

    def close_serial(self):
        self.receive_thread_stop.set()  # Signal the receive thread to stop
        self.receive_thread.join()  # Wait for the receive thread to stop
        if self.serial_port.is_open:
            self.serial_port.close()


class App:
    def __init__(self, master, bluetooth_interface):
        self.master = master
        self.bluetooth_interface = bluetooth_interface
        self.previous_messages = []
        self.previous_received_messages = []
        self.current_message_index = -1

        self.frame = tk.Frame(self.master)
        self.frame.pack()

        self.message_label = tk.Label(self.frame, text="Message:")
        self.message_label.grid(row=0, column=0)

        self.message_entry = tk.Entry(self.frame)
        self.message_entry.grid(row=0, column=1)
        # Bind Return key to send_message_event
        self.message_entry.bind("<Return>", self.send_message_event)

        self.send_button = tk.Button(
            self.frame, text="Send", command=self.send_message)
        self.send_button.grid(row=0, column=2)

        self.message_history_label = tk.Label(
            self.frame, text="Message History:")
        self.message_history_label.grid(
            row=1, column=0, columnspan=3, sticky="w")

        self.message_history_text = tk.Text(self.frame, height=10, width=40)
        self.message_history_text.grid(row=2, column=0, columnspan=3)

        self.received_message_label = tk.Label(
            self.frame, text="Received Message:")
        self.received_message_label.grid(row=3, column=0, sticky="w")

        self.received_message_text = tk.Text(self.frame, height=10, width=150)
        self.received_message_text.grid(row=4, column=0, columnspan=3)

        # Bind up and down arrow keys to load previous messages
        self.master.bind("<Up>", self.load_previous_message)
        self.master.bind("<Down>", self.load_next_message)

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


def main():
    bluetooth_port = '/dev/cu.HC-06'  # for Mac
    # bluetooth_port = 'COM6'  # for Windows

    root = tk.Tk()
    root.title("WIPER CONTROL")
    app = App(root, None)  # Pass None initially for the BluetoothInterface

    bluetooth_interface = BluetoothInterface(
        port=bluetooth_port, baudrate=9600, app=app)
    # Update app's BluetoothInterface reference
    app.bluetooth_interface = bluetooth_interface

    # Calculate the position to center the window
    window_width = 1200  # Adjust width as needed
    window_height = 400  # Adjust height as needed
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    x_coordinate = (screen_width - window_width) // 2
    y_coordinate = (screen_height - window_height) // 2

    # Set window dimensions and position
    root.geometry(
        f"{window_width}x{window_height}+{x_coordinate}+{y_coordinate}")
    # Properly close the GUI window
    root.protocol("WM_DELETE_WINDOW", root.quit)
    root.mainloop()


if __name__ == "__main__":
    main()
