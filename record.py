"""
record.py

Streams robot states and records robot demonstrations for playback. Simple socket-based interface, spins up
`readState` C++ code on start-up.

Assumes Kinesthetic Demonstrations (experts directly manipulate Panda arm in `backdrive` mode), via native Libfranka
(no ROS!) controller. Inputs for recording are provided via Joystick input.

Note: Assumes that the robot always starts at the "home" position (`go_JointPosition` w/ default args).
"""
from subprocess import call, Popen
from tap import Tap

import numpy as np
import os
import socket
import time

# Suppress PyGame Import Text
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "hide"
import pygame


# CONSTANTS
STEP_TIME = 0.1                                 # Record Joint State every 0.1 Seconds -- IMPORTANT!


class ArgumentParser(Tap):
    # fmt: off

    # Mode
    mode: str = "stream"                        # Record mode in < stream | record >

    # Connection Parameters
    robot_ip: str = "172.16.0.3"                # IP of Robot to connect to in 172.16.0.<2, 3>
    port: int = 8080                            # Port on which to spawn socket

    # fmt: on


def connect2robot(port):
    """ Establish a Socket Connection and initiates handshake with the robot """
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("localhost", port))
    s.listen()
    conn, addr = s.accept()
    return conn


def listen2robot(conn):
    """ Polling Function -- Read from the socket, and attempt to parse out `state` string """
    state_length = 7 + 7 + 7
    message = str(conn.recv(1024))[2:-2]
    state_str = list(message.split(","))
    for idx in range(len(state_str)):
        if state_str[idx] == "s":
            state_str = state_str[idx + 1 : idx + 1 + state_length]
            break
    try:
        state_vector = [float(item) for item in state_str]
        assert len(state_vector) == state_length
    except (AssertionError, ValueError):
        return None

    # State Vector is Concatenated Joint Positions [0 - 7], Joint Velocities [8 - 14], and Joint Torques [15 - 21]
    state_vector = np.asarray(state_vector)[0:21]
    return state_vector


def get_state(conn):
    """ Get state by continuously reading from the socket until valid! """
    while True:
        state = listen2robot(conn)
        if state is not None:
            break
    return state


def record():
    # Parse Arguments
    print("[*] Starting Up...")
    args = ArgumentParser().parse_args()

    # Open Socket Connection to Robot
    print("[*] Opening Socket Connection to Robot...")
    conn = connect2robot(args.port)

    # Note --> will Block here until handshake from Robot side...
    print("[*] Connection Established... entering stream/record mode!")

    # Switch on Mode
    if args.mode in ["stream"]:
        # Enter Loop and Print Joint States (on a set interval)
        while True:
            # Read State and get Joint Values
            state = get_state(conn)[:7]

            # Print State every second
            print(f"Joint State: {' | '.join(list(state))}\n")

            # Sleep for a Second
            time.sleep(1)


if __name__ == "__main__":
    record()
