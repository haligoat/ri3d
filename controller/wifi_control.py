# pip install pynput

import math
import socket
import threading
import time

from pynput import keyboard

ECHO_IP = "192.168.4.1"
ECHO_PORT = 8888
SPEED = 12

# WiFiServerBridge drops the connection (and hard-stops the motors) after 2 s
# without a packet, so a held key has to be re-sent, not sent once. The
# odometry motion model also assumes the command it was last told about is
# still current -- a stale command makes the filter predict motion the robot
# is not making.
HEARTBEAT_HZ = 20

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.5)

held_keys = set()
running = True
latest_pose = None


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
    sock.sendto(f"{x},{y},{turn}".encode(), (ECHO_IP, ECHO_PORT))


def heartbeat_loop():
    period = 1.0 / HEARTBEAT_HZ
    while running:
        send_state()
        time.sleep(period)


def receive_loop():
    global latest_pose
    while running:
        try:
            data, _ = sock.recvfrom(1500)
        except socket.timeout:
            continue
        except OSError:
            return
        msg = data.decode(errors="replace").strip()
        if not msg.startswith("ODOM,"):
            continue
        parts = msg.split(",")
        if len(parts) != 7:
            continue
        try:
            latest_pose = tuple(float(v) for v in parts[1:6]) + (parts[6] == "1",)
        except ValueError:
            continue


def display_loop():
    while running:
        if latest_pose is not None:
            x, y, theta, vx, vy, still = latest_pose
            speed = math.hypot(vx, vy)
            flag = "PARKED" if still else "      "
            print(
                f"\r x={x:+7.3f} m  y={y:+7.3f} m  θ={theta:+7.1f}°"
                f"  |v|={speed:5.2f} m/s  {flag}",
                end="",
                flush=True,
            )
        time.sleep(0.1)


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
        k = None
    if k in held_keys:
        held_keys.remove(k)
        send_state()
    if key == keyboard.Key.esc:
        return False


print(f"Sending commands to {ECHO_IP}:{ECHO_PORT}")
print("Hold w/a/s/d to drive, q/e to turn, release to stop, ESC to quit.")
print("Live odometry (pose is relative to where the robot booted):\n")

threads = [
    threading.Thread(target=heartbeat_loop, daemon=True),
    threading.Thread(target=receive_loop, daemon=True),
    threading.Thread(target=display_loop, daemon=True),
]
for t in threads:
    t.start()

try:
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()
finally:
    running = False
    held_keys.clear()
    sock.sendto(b"0,0,0", (ECHO_IP, ECHO_PORT))
    time.sleep(0.1)
    sock.close()
    print()
