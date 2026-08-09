# pip install pynput

import socket
from pynput import keyboard

ECHO_IP = "192.168.4.1"
ECHO_PORT = 8888
SPEED = 50

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

held_keys = set()

def compute_state():
    x = 0
    y = 0
    turn = 0
    if 'w' in held_keys:
        y += SPEED
    if 's' in held_keys:
        y -= SPEED
    if 'a' in held_keys:
        x -= SPEED
    if 'd' in held_keys:
        x += SPEED
    if 'q' in held_keys:
        turn -= SPEED
    if 'e' in held_keys:
        turn += SPEED
    return x, y, turn

def send_state():
    x, y, turn = compute_state()
    message = f"{x},{y},{turn}"
    sock.sendto(message.encode(), (ECHO_IP, ECHO_PORT))
    print("Sent:", message)

def on_press(key):
    try:
        k = key.char
    except AttributeError:
        return
    if k in ('w', 'a', 's', 'd', 'q', 'e') and k not in held_keys:
        held_keys.add(k)
        send_state()

def on_release(key):
    try:
        k = key.char
    except AttributeError:
        return
    if k in held_keys:
        held_keys.remove(k)
        send_state()
    if key == keyboard.Key.esc:
        return False

print(f"Sending commands to {ECHO_IP}:{ECHO_PORT}")
print("Hold w/a/s/d to drive, q/e to turn, release to stop, ESC to quit.")

with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()

sock.sendto(b"0,0,0", (ECHO_IP, ECHO_PORT))
