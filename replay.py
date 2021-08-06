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

# Suppress PyGame Import Text
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "hide"
import pygame


class ArgumentParser(Tap):
    # fmt: off

    # Save Parameters
    name: str                                               # Name of the Demonstrations
    demonstration_path: str                                 # Path to Demonstration Storage Directory

    # Mode
    mode: str = "naive"                                     # Mode for Playback in < naive | staggered | cubic >

    # Selection Parameters
    index: int = 0                                          # Index of demonstration for playback (default: 0)

    # fmt: on


def playback():
    # Parse Arguments
    print("[*] Starting Up...")
    args = ArgumentParser().parse_args()


if __name__ == "__main__":
    playback()
