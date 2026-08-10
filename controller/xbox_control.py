# pip install pygame

import math
import os
import socket
import threading
import time

import pygame

# The board is a WiFi station on the house network now, not its own SoftAP, so
# this is a DHCP address and can move. Override without editing the file:
#   ECHO_IP=192.168.68.99 python3 xbox_control.py
ECHO_IP = os.environ.get("ECHO_IP", "192.168.68.85")
ECHO_PORT = 8888
SPEED = 100          # max magnitude sent for x/y/turn (matches drive()'s +-255-ish range, tune as needed)
DEADZONE = 0.15       # ignore stick drift near center
HEARTBEAT_HZ = 20      # WiFiServerBridge drops the connection after 2 s without a packet

# Right bumper index under xpadneo's 360-style mapping (0=A 1=B 2=X 3=Y 4=LB
# 5=RB). Override if your driver enumerates differently -- run the axis/button
# sampler and watch which index lights up.
RB_BUTTON = int(os.environ.get("RB_BUTTON", "5"))

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.5)

# Local telemetry dashboard (telemetry_server.py). Loopback only.
DASHBOARD_ADDR = (os.environ.get("DASHBOARD_HOST", "127.0.0.1"),
                  int(os.environ.get("DASHBOARD_PORT", "9999")))
telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

running = True
latest_pose = None
state_lock = threading.Lock()
current_state = (0, 0, 0, 0)


def apply_deadzone(v):
    """Deadzone with rescaling, so output is proportional to stick travel.

    A plain cutoff makes power jump straight to DEADZONE*SPEED the instant the
    stick clears the threshold -- 15% power from a barely-moved stick. Rescaling
    the remaining range back to 0..1 means power ramps from zero at the edge of
    the deadzone up to full at the rail.
    """
    magnitude = abs(v)
    if magnitude < DEADZONE:
        return 0.0
    scaled = (magnitude - DEADZONE) / (1.0 - DEADZONE)
    return scaled if v > 0 else -scaled


def rumble(joystick, low, high, ms):
    """Best-effort haptics.

    SDL's rumble is unsupported on some drivers and raises rather than no-ops.
    A missing buzz is cosmetic -- it must never take the driver station down
    while the robot is moving.
    """
    try:
        joystick.rumble(low, high, ms)
    except Exception:
        pass


def send_state():
    with state_lock:
        x, y, turn, aux = current_state
    sock.sendto(f"{x},{y},{turn},{aux}".encode(), (ECHO_IP, ECHO_PORT))


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

        # Mirror to the local dashboard. Deliberately forwarded from here
        # rather than having the web server talk to the board: the firmware
        # replies to whichever client sent last, so a second UDP client would
        # steal telemetry from this process -- and its packets would satisfy
        # the firmware's clientConnected() check, keeping the robot armed with
        # no driver station attached.
        try:
            telemetry_sock.sendto(msg.encode(), DASHBOARD_ADDR)
        except OSError:
            pass  # dashboard down is not a driving problem


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


def main():
    global running, current_state

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No controller detected. Plug in / pair your Xbox controller and try again.")
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Using controller: {joystick.get_name()}")

    # Confirms haptics work before anyone relies on them for feedback.
    rumble(joystick, 0.5, 0.5, 250)

    print(f"Sending commands to {ECHO_IP}:{ECHO_PORT}")
    print("Left stick = drive, right stick X = turn. Ctrl+C to quit.")
    print(f"Right bumper (button {RB_BUTTON}) = motor 5 full speed while held.")
    print("Live odometry (pose is relative to where the robot booted):\n")

    threads = [
        threading.Thread(target=heartbeat_loop, daemon=True),
        threading.Thread(target=receive_loop, daemon=True),
        threading.Thread(target=display_loop, daemon=True),
    ]
    for t in threads:
        t.start()

    prev_aux = 0

    try:
        while True:
            pygame.event.pump()

            # Xbox controller axes (pygame): 0 = left stick X, 1 = left stick Y
            # (inverted, up is negative), 3 = right stick X. Confirm mapping
            # with joystick.get_name() / print(axis values) if your OS/driver
            # orders them differently.
            raw_x = joystick.get_axis(0)
            raw_y = -joystick.get_axis(1)
            raw_turn = joystick.get_axis(3)

            x = int(apply_deadzone(raw_x) * SPEED)
            y = int(apply_deadzone(raw_y) * SPEED)
            turn = int(apply_deadzone(raw_turn) * SPEED)

            # Motor 5 runs flat out while the right bumper is held. Sent as a
            # level, not an edge, so a dropped packet cannot latch it on -- the
            # firmware also kills it on link loss.
            aux = 1 if joystick.get_button(RB_BUTTON) else 0

            if aux != prev_aux:
                # Short buzz on engage, softer one on release, so the driver
                # can feel the state change without looking away from the robot.
                rumble(joystick, 0.0, 0.8, 120) if aux else rumble(joystick, 0.4, 0.0, 80)
                prev_aux = aux

            with state_lock:
                current_state = (x, y, turn, aux)

            time.sleep(1 / 60)
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        with state_lock:
            current_state = (0, 0, 0, 0)
        # Explicit aux=0 so motor 5 stops on Ctrl+C rather than waiting for the
        # firmware's 2 s client timeout.
        sock.sendto(b"0,0,0,0", (ECHO_IP, ECHO_PORT))
        time.sleep(0.1)
        sock.close()
        pygame.quit()
        print()


if __name__ == "__main__":
    main()
