"""
replay.py

Plays back a demonstration in one of two modes (three if Sidd has the time to implement it):
    - Naive Playback --> push recorded joint velocities straight to robot in a loop!
    - Staggered Playback --> push recorded joint velocities based on a time offset!
    - Cubic Interpolated Playback --> Interpolate the trajectory across a continuous [DURATION] and feed states based
      on ( interpolated-position(t) - q < current state > ).
"""
from matplotlib import pyplot as plt
from tap import Tap

import numpy as np
import os
import pickle
import socket
import time


# Constants
STEP_TIME = 0.1
EFFECTIVE_HERTZ = 30


class ArgumentParser(Tap):
    # fmt: off

    # Save Parameters
    name: str                                               # Name of the Demonstrations
    demonstration_path: str                                 # Path to Demonstration Storage Directory

    # Mode
    mode: str = "no-sleep"                                  # Mode for Playback in < naive | staggered | cubic >

    # Selection Parameters
    index: int = 0                                          # Index of demonstration for playback (default: 0)

    # fmt: on


# Robot Control Functions
def connect2robot(port):
    """ Open a Socket Connection to the Low-Level Controller """
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("localhost", port))
    s.listen()
    conn, addr = s.accept()
    return conn


def send2robot(conn, qdot, limit=1.5):
    """ Send a Joint Velocity Command to the Low Level Controller """
    qdot = np.asarray(qdot)
    scale = np.linalg.norm(qdot)
    if scale > limit:
        qdot = np.asarray([qdot[i] * limit / scale for i in range(7)])
    send_msg = np.array2string(qdot, precision=5, separator=",", suppress_small=True)[1:-1]
    send_msg = "s," + send_msg + ","
    conn.send(send_msg.encode())


def listen2robot(conn):
    """ Read in the State Information Sent Back by Low Level Controller """
    state_length = 7 + 7 + 7 + 42
    state_message = str(conn.recv(2048))[2:-2]
    state_str = list(state_message.split(","))
    for idx in range(len(state_str)):
        if state_str[idx] == "s":
            state_str = state_str[idx + 1 : idx + 1 + state_length]
            break

    try:
        state_vector = [float(item) for item in state_str]
        assert len(state_vector) == state_length
    except (ValueError, AssertionError):
        return None

    state = {
        "q": np.asarray(state_vector[0:7]),
        "dq": np.asarray(state_vector[7:14]),
        "tau": np.asarray(state_vector[14:21]),
        "J": np.array(state_vector[21:]).reshape(7, 6).T,
    }

    return state


def get_state(conn):
    """ Get state by continuously reading from the socket until valid! """
    while True:
        state = listen2robot(conn)
        if state is not None:
            break
    return state


def replay():
    # Parse Arguments
    print("[*] Starting Up...")
    args = ArgumentParser().parse_args()

    print("[*] Connecting to Low-Level Controller...")
    conn = connect2robot(8080)

    # Load the List of Saved Demonstrations
    print("[*] Loading Demonstrations...")
    with open(os.path.join(args.demonstration_path, "%s.pkl" % args.name), "rb") as f:
        demonstrations = pickle.load(f)
    print(f"Number of demonstrations = {len(demonstrations)}")

    # Get States for Demonstration
    states = demonstrations[args.index]
    true_joints, target_joints, idx = np.zeros((len(states), 7)), np.zeros((len(states), 7)), 0
    for (state, elapsed) in states:
        print(f"Processing Index: {idx}")

        # Check if time between consecutive states is at least 0.1 seconds (assumes STEP_TIME in record.py = 0.1)
        assert elapsed >= STEP_TIME, "Time between consecutive states is less than 0.1 seconds"

        # Check if the current state represents a valid 1-dimensional numpy array with 21 elements.
        #       =>> joint positions, joint velocities, and joint torques (3 x 7D)
        assert state.ndim == 1 and state.size == 21, "Invalid state (Current state does not have 21 elements)!"

        # Get Current Robot State, and "Target" Robot State
        true_joints[idx] = get_state(conn)["q"][:7]
        target_joints[idx] = state[:7]

        # Extract velocity to send to robot
        qdot = state[7:14]

        # Naive --> Extract Velocity from indexed state, and feed it to the robot immediately!
        if args.mode in ["naive"]:
            # Extract Velocity and Send to Robot
            send2robot(conn, qdot)

        # Staggered --> Extract Velocity from indexed state, and wait "elapsed" time before feeding to the robot
        elif args.mode in ["staggered"]:
            # Sleep for Elapsed Time
            time.sleep(elapsed)

            for _ in range(EFFECTIVE_HERTZ):
                # Extract Velocity and Send to Robot
                send2robot(conn, qdot)

        idx += 1

    # Close Connection
    conn.close()

    # Compute and Plot Distances over Time...
    mismatch = np.linalg.norm(target_joints - true_joints, axis=1)
    plt.plot(mismatch)
    plt.xlabel("State Index")
    plt.ylabel("Euclidean Distance Between Target Joints and Replayed Joints")
    plt.ylim([0, 5])
    plt.show()


if __name__ == "__main__":
    replay()
