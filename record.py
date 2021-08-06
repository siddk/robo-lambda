"""
record.py

Streams robot states and records robot demonstrations for playback. Simple socket-based interface, spins up
`readState` C++ code on start-up.

Assumes Kinesthetic Demonstrations (experts directly manipulate Panda arm in `backdrive` mode), via native Libfranka
(no ROS!) controller. Inputs for recording are provided via Joystick input.

Note: Assumes that the robot always starts at the "home" position (`go_JointPosition` w/ default args).
"""
from subprocess import Popen, call
from tap import Tap

import numpy as np
import os
import pickle
import socket
import time

# Suppress PyGame Import Text
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "hide"
import pygame


# CONSTANTS
STEP_TIME = 0.1                                             # Record Joint State every 0.1 Seconds -- IMPORTANT!


class ArgumentParser(Tap):
    # fmt: off

    # Mode
    mode: str = "stream"                                    # Record mode in < stream | record >

    # Connection Parameters
    robot_ip: str = "172.16.0.3"                            # IP of Robot to connect to in 172.16.0.<2, 3>
    port: int = 8080                                        # Port on which to spawn socket

    # Save Parameters (for recording)
    name: str                                               # Name of the Demonstrations to be Collected
    demonstration_path: str                                 # Path to Demonstration Storage Directory

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

    # Switch on Mode
    if args.mode in ["stream"]:
        # Open Socket Connection to Robot
        print("[*] Opening Socket Connection to Robot...")
        conn = connect2robot(args.port)

        # Note --> will Block here until handshake from Robot side...
        print("[*] Connection Established... entering stream/record mode!")

        # Enter Loop and Print Joint States (on a set interval)
        start_time = time.time()
        while True:
            # Read State and get Joint Values
            state = get_state(conn)[:7]

            # Print State every second
            if time.time() - start_time > 1.0:
                print(f"Joint State: {' | '.join(list(map(str, state)))}\n")
                start_time = time.time()

            # Socket Gotcha -- Sleep for a Second
            # time.sleep(1)``

    # Record a Demonstration
    elif args.mode in ["record"]:
        # Simple Joystick (Logitech Controller) Wrapper --> [A] starts recording, [B] stops recording,
        #                                                   [START] resets robots, [BACK] ends session
        class Joystick:
            def __init__(self):
                pygame.init()
                self.gamepad = pygame.joystick.Joystick(0)
                self.gamepad.init()

            def input(self):
                pygame.event.get()
                A, B = self.gamepad.get_button(0), self.gamepad.get_button(1)
                START, BACK = self.gamepad.get_button(7), self.gamepad.get_button(6)
                return A, B, START, BACK

        # Establish Initial Connection to Robot --> First connect, then spawn LibFranka Low-Level Process (./read_State)
        print("[*] Initializing Connection to Robot and Starting Read-State Loop...")
        lf_process = Popen(f"~/libfranka/build/examples/read_State {args.robot_ip} {args.port}", shell=True)
        conn = connect2robot(args.port)

        print("[*] Dropping into Demonstration Loop w/ Input Device Controller = Joystick\n")
        print("[*] (A) to start recording, (B) to stop, (START) to reset, and (BACK) to end...")
        joystick, demonstrations = Joystick(), []
        while True:
            start, _, reset, end = joystick.input()

            # Stop Collecting Demonstrations
            if end:
                print("[*] Exiting Interactive Loop...")
                break

            # Collect a Demonstration <----> ENSURE ROBOT IS BACK-DRIVEABLE (Hit E-Stop)
            elif start:
                # Initialize Trackers
                initial_img, demo = None, []

                # Drop into Collection Loop, recording States + Frames at STEP_TIME Intervals
                print(f"[*] Starting Demonstration {len(demonstrations) + 1} Recording...")
                start_time = time.time()
                while True:
                    # Read State
                    state = get_state(conn)

                    # Get Current Time and Only Record if Difference between Start and Current is > Step
                    curr_time = time.time()
                    if curr_time - start_time >= STEP_TIME:
                        # Add a Tuple of current state and time elapsed between previous and current state
                        print(f"Recorded State: {' | '.join(list(map(str, state[:7])))}\n")
                        demo.append((state, curr_time - start_time))

                        # Reset Start Time!
                        start_time = time.time()

                    # Get Joystick Input
                    _, stop, _, _ = joystick.input()
                    if stop:
                        print("[*] Stopped Recording!")
                        break

                # Add demo to list of demonstrations!
                demonstrations.append(demo)

                # Log
                print(f"[*] Demonstration {len(demonstrations)}\tTotal Steps: {len(demo)}")

            elif reset:
                # Kill Existing Socket
                conn.close()

                # Kill LibFranka Read State
                lf_process.kill()

                # Call Reset to Home Position --> libfranka/build/examples/go_JointPosition robot_ip
                call(f"~/libfranka/build/examples/go_JointPosition {args.robot_ip}", shell=True)

                # Re-initialize LF ReadState & Socket Connection
                lf_process = Popen(f"~/libfranka/build/examples/read_State {args.robot_ip} {args.port}", shell=True)
                conn = connect2robot(args.port)

        # Serialize and Save Demonstrations + Images
        if not os.path.exists(args.demonstration_path):
            os.makedirs(args.demonstration_path)

        with open(os.path.join(args.demonstration_path, "%s.pkl" % args.name), "wb") as f:
            pickle.dump(demonstrations, f)

        # Cleanup
        print("[*] Shutting Down...")
        lf_process.kill()
        conn.close()


if __name__ == "__main__":
    record()
