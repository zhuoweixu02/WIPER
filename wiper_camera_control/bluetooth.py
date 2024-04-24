import serial
import time

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
    
def msg_read(ser):
    msg = ser.readline().decode().strip()
    print(f"Message: {msg}")
    return msg