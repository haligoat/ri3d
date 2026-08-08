# pip install pyserial pynput

import serial
import time
from pynput import keyboard

SERIAL_PORT = "/dev/cu.usbmodem1101"   # update to match your board's port
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # give the board time to reset after opening the port

VALID_KEYS = ('w', 'a', 's', 'd')

def on_press(key):
    try:
        k = key.char
    except AttributeError:
        return

    if k in VALID_KEYS:
        ser.write(k.encode())
        print("Sent:", k)

def on_release(key):
    if key == keyboard.Key.esc:
        return False  # stop listener

print("Press w/a/s/d to send commands. ESC to quit.")

with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()

ser.close()
