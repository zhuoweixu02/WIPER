import serial
import time


def bluetooth_init():
    bluetooth_port = '/dev/cu.HC-06'  # for Mac
    # bluetooth_port = 'COM6'  # for Windows
    baud_rate = 9600
    ser = serial.Serial(bluetooth_port, baud_rate)
    time.sleep(1)  # Wait for the connection to establish
    print(f"Connected to {bluetooth_port} at {baud_rate} baud.")
    return ser


def cmd_write(carx, cary, targetx, targety, mode, power):
    cmd = f"{carx},{cary}|{targetx},{targety}|{mode}|{power}\n"
    print(f"Command: {cmd}")
    ser.write(cmd.encode())
    time.sleep(0.5)  # Wait for the command to be executed


if "__main__" == __name__:
    ser = bluetooth_init()
    # while(1):
    #     cmd = input("Enter command (type 'q' to quit): ")
    #     if cmd.lower() == 'q':
    #         break
    #     ser.write(cmd.encode())
    a = 1
    b = 1
    while (1):
        cmd_write(a, 2, 3, 4, 2+b, 1)
        a += 1
        b = -b
        if a > 100:
            break
    # while(1):
    #     cmd = input("type 'q' to quit: ")
    #     if cmd.lower() == 'q':
    #         break
    ser.close()
