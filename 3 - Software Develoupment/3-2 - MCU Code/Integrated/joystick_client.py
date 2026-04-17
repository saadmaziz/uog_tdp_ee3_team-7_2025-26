"""
Logitech Extreme 3D Pro → ESP32 WiFi Bridge
============================================
Reads the joystick through Windows DirectInput (via pygame) and sends
a compact UDP packet to the ESP32 server on every poll cycle.

UDP Packet Layout  (18 bytes, little-endian):
  Bytes  0-1  : uint16  button bitmask  (bits 0-11 = buttons 1-12)
  Bytes  2-5  : float32 roll   (X axis,    -1.0 … +1.0)
  Bytes  6-9  : float32 pitch  (Y axis,    -1.0 … +1.0)
  Bytes 10-13 : float32 yaw    (Z-rotate,  -1.0 … +1.0)
  Bytes 14-17 : float32 throttle (slider,  -1.0 … +1.0, -1=full fwd)

Requirements
------------
  pip install pygame

Usage
-----
  1. Edit ESP32_IP and ESP32_PORT below to match your ESP32.
  2. Plug in the joystick BEFORE running.
  3. python joystick_client.py
"""

import pygame
import socket
import struct
import time
import sys

# ── Configuration ────────────────────────────────────────────────────────────
ESP32_IP   = "192.168.4.1"     # ESP32 Access Point fixed IP (never changes)
ESP32_PORT = 4210              # must match ESP32 sketch
POLL_HZ    = 50                # packets per second (50 Hz is plenty)
# ─────────────────────────────────────────────────────────────────────────────

PACKET_FMT = "<Hffff"          # uint16 + 4× float32  = 18 bytes
POLL_SEC   = 1.0 / POLL_HZ


def find_extreme3d(joy_count: int) -> int | None:
    """Return the index of the first Extreme 3D Pro found, or None."""
    for i in range(joy_count):
        j = pygame.joystick.Joystick(i)
        name = j.get_name().lower()
        if "extreme" in name or "3d pro" in name or "extreme 3d" in name:
            return i
    # Fall back to joystick 0 if only one is connected
    if joy_count == 1:
        print("[WARN] Could not identify Extreme 3D Pro by name; using joystick 0.")
        return 0
    return None


def axis_value(joystick: pygame.joystick.JoystickType, axis: int) -> float:
    """Return axis value clamped to [-1, +1], or 0.0 if the axis doesn't exist."""
    try:
        return max(-1.0, min(1.0, joystick.get_axis(axis)))
    except pygame.error:
        return 0.0


def build_packet(joystick: pygame.joystick.JoystickType) -> bytes:
    """
    Extreme 3D Pro axis mapping (pygame / WinMM):
      Axis 0  → X  (roll,     left/right)
      Axis 1  → Y  (pitch,    forward/back)
      Axis 2  → Z  (twist/yaw, rotate grip)
      Axis 3  → Throttle slider
    """
    # --- analogue axes ---
    roll     = axis_value(joystick, 0)
    pitch    = axis_value(joystick, 1)
    yaw      = axis_value(joystick, 2)
    throttle = axis_value(joystick, 3)

    # --- buttons (up to 16 bits) ---
    btn_mask: int = 0
    num_buttons = min(joystick.get_numbuttons(), 16)
    for b in range(num_buttons):
        if joystick.get_button(b):
            btn_mask |= (1 << b)

    return struct.pack(PACKET_FMT, btn_mask, roll, pitch, yaw, throttle)


def print_state(btn_mask: int, roll: float, pitch: float,
                yaw: float, throttle: float) -> None:
    active = [str(b + 1) for b in range(12) if btn_mask & (1 << b)]
    btns   = ",".join(active) if active else "none"
    print(
        f"\rRoll:{roll:+.3f}  Pitch:{pitch:+.3f}  "
        f"Yaw:{yaw:+.3f}  Thr:{throttle:+.3f}  "
        f"Btns:[{btns:15s}]",
        end="", flush=True,
    )


def main() -> None:
    # ── Init pygame joystick subsystem ──────────────────────────────────────
    pygame.init()
    pygame.joystick.init()

    count = pygame.joystick.get_count()
    if count == 0:
        print("[ERROR] No joystick detected. Plug in the Extreme 3D Pro and retry.")
        sys.exit(1)

    idx = find_extreme3d(count)
    if idx is None:
        print(f"[ERROR] Extreme 3D Pro not found among {count} device(s).")
        for i in range(count):
            j = pygame.joystick.Joystick(i)
            print(f"  [{i}] {j.get_name()}")
        sys.exit(1)

    joystick = pygame.joystick.Joystick(idx)
    joystick.init()
    print(f"[OK] Using joystick [{idx}]: {joystick.get_name()}")
    print(f"     Axes: {joystick.get_numaxes()}  "
          f"Buttons: {joystick.get_numbuttons()}  "
          f"Hats: {joystick.get_numhats()}")

    # ── UDP socket ──────────────────────────────────────────────────────────
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[OK] Sending UDP → {ESP32_IP}:{ESP32_PORT} at {POLL_HZ} Hz\n")

    # ── Main loop ───────────────────────────────────────────────────────────
    clock = pygame.time.Clock()
    try:
        while True:
            pygame.event.pump()          # keep WinMM happy (updates axes/buttons)

            pkt = build_packet(joystick)
            sock.sendto(pkt, (ESP32_IP, ESP32_PORT))

            # Unpack for local console display
            btn_mask, roll, pitch, yaw, throttle = struct.unpack(PACKET_FMT, pkt)
            print_state(btn_mask, roll, pitch, yaw, throttle)

            clock.tick(POLL_HZ)

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    finally:
        sock.close()
        pygame.quit()


if __name__ == "__main__":
    main()
