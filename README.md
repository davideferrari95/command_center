# Command Feasibility Checker and ROS Command GUI

This package contains two main components:

1. A Flask server that checks the feasibility of an action based on a `Command` object.
2. A GUI script that reads commands from a ROS topic and prints them on the screen.

## Requirements

- Python 3.8.10+
- Flask
- ROS Noetic

### Dependencies

- alexa_conversation → [GitHub Source](https://github.com/davideferrari95/alexa_collaborative_conversation/tree/chimera)

## Installation

- Clone and build the repository:

        cd ../ros_workspace/src
        git clone https://github.com/tuo-utente/command_center.git
        cd ..
        catkin_make

- Install the required Python packages:

        pip install -r requirements.txt

## Usage

- Run the Flask server:

        python command_server.py

- Launch the GUI:

        roslaunch command_center command_center.launch
