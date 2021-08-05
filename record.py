"""
record.py

Streams robot states and records robot demonstrations for playback. Simple socket-based interface, spins up
`readState` C++ code on start-up.

Assumes Kinesthetic Demonstrations (experts directly manipulate Panda arm in `backdrive` mode), via native Libfranka
(no ROS!) controller. Inputs for recording are provided via Joystick input.

Note: Assumes that the robot always starts at the "home" position (`go_JointPosition` w/ default args).
"""



