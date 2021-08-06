"""
teleoperate.py

Code for teleoperating a Franka Emika Panda Arm with a two degree-of-freedom (2-DoF) controller (e.g.,
a single joystick). As this is end-effector control, this translates to:
    - 1 Mode for "X", "Y"
    - 1 Mode for "Z", "Roll"
    - 1 Mode for "Pitch", "Yaw"

While there are other possible ways to instantiate this (multi-joysticks w/ triggers, mobile/haptic interfaces), the
basic mapping of end-effector velocity to joint velocity remains the same, via Resolved Rates IK.
"""
import socket

import numpy as np
import os
import time

# Suppress PyGame Import Text
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame

# End-Effector Map
END_EFF_DIM = 6


class JoystickControl(object):
    def __init__(self):
        pygame.init()
        self.gamepad = pygame.joystick.Joystick(0)
        self.gamepad.init()
        self.DEADBAND, self.AXIS_RANGE = 0.1, 2

    def input(self):
        pygame.event.get()
        # 2D Control: We should get both axes 0, 1
        zs = []
        for i in range(self.AXIS_RANGE):
            z = self.gamepad.get_axis(i)
            if abs(z) < self.DEADBAND:
                z = 0.0
            zs.append(z)

        # Buttons
        a, stop = self.gamepad.get_button(0), self.gamepad.get_button(7)
        return zs, a, stop


# End-Effector Control Functions (Resolved Rates IK)
def resolved_rates(xdot, j, scale=1.0):
    """ Compute the pseudo-inverse of the Jacobian to map the delta in end-effector velocity to joint velocities """
    j_inv = np.linalg.pinv(j)
    return [qd * scale for qd in (np.dot(j_inv, xdot))]


# Robot Control Functions
def connect2robot(PORT):
    """ Open a Socket Connection to the Low-Level Controller """
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('localhost', PORT))
    s.listen()
    conn, addr = s.accept()
    return conn


def send2robot(conn, qdot, limit=1.5):
    """ Send a Joint Velocity Command to the Low Level Controller """
    qdot = np.asarray(qdot)
    scale = np.linalg.norm(qdot)
    if scale > limit:
        qdot = np.asarray([qdot[i] * limit / scale for i in range(7)])
    send_msg = np.array2string(qdot, precision=5, separator=',', suppress_small=True)[1:-1]
    send_msg = "s," + send_msg + ","
    conn.send(send_msg.encode())


def listen2robot(conn):
    """ Read in the State Information Sent Back by Low Level Controller """
    state_length = 7 + 7 + 7 + 42
    state_message = str(conn.recv(2048))[2:-2]
    state_str = list(state_message.split(","))
    for idx in range(len(state_str)):
        if state_str[idx] == "s":
            state_str = state_str[idx + 1:idx + 1 + state_length]
            break

    try:
        state_vector = [float(item) for item in state_str]
        assert(len(state_vector) == state_length)
    except (ValueError, AssertionError):
        return None

    state = {
        'q': np.asarray(state_vector[0:7]),
        'dq': np.asarray(state_vector[7:14]),
        'tau': np.asarray(state_vector[14:21]),
        'J': np.array(state_vector[21:]).reshape(7, 6).T
    }

    return state


def get_state(conn):
    """ Get state by continuously reading from the socket until valid! """
    while True:
        state = listen2robot(conn)
        if state is not None:
            break
    return state


# Main Teleoperation Code
def main():
    # Parse Arguments
    print('[*] Starting up...')
    print("\t[*] \"I'm rooting for the machines.\" (Claude Shannon)")

    # Connect to Gamepad, Robot
    print('\n[*] Connecting to Gamepad...')
    joystick = JoystickControl()

    print('[*] Connecting to Low-Level Controller...')
    conn = connect2robot(8080)

    # Enter Control Loop
    start_time, toggle, pressed, pressed_time = time.time(), 0, False, None
    try:
        while True:
            # Read the Robot State
            state = get_state(conn)

            # Measure Joystick Input
            z, a, stop = joystick.input()
            if stop:
                print("[*] You pressed START, so saving & exiting...")
                break

            # Otherwise - End Effector Control
            else:
                # If A-Button Pressed, switch mode
                if a:
                    if not pressed:
                        pressed, pressed_time = True, time.time()
                        # since we are using 2-DoF, skip to the axis after the next one
                        toggle = (toggle + 2) % END_EFF_DIM

                    # Make sure not "holding" button on press
                    if time.time() - pressed_time > 0.25:
                        pressed = False

                # Otherwise --> Control w/ Latent Dim
                else:
                    # Set xdot[toggle] to be user's input on first axis, xdot[toggle + 1] to be input on second axis
                    xdot = np.zeros(END_EFF_DIM)
                    xdot[toggle] = z[0]
                    xdot[toggle + 1] = z[1]

                    # Resolved Rate Motion Control
                    qdot = resolved_rates(xdot, state['J'], scale=0.5)

                    # Send joint-velocity command
                    send2robot(conn, qdot)

    except (KeyboardInterrupt, ConnectionResetError, BrokenPipeError):
        # Just don't crash the program on Ctrl-C or Socket Error (Controller Death)
        pass


if __name__ == "__main__":
    main()
