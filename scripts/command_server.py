#!/usr/bin/env python3

import sys, rospkg
from flask import Flask, request, jsonify

# Import Parent Folders
sys.path.append(f'{rospkg.RosPack().get_path("alexa_conversation")}/AzureFunctions')

# Import Commands
from SkillBackend.Utils.command_list  import *

# Create the Flask App
app = Flask(__name__)

# Check Action Route
@app.route('/check_action', methods=['POST'])
def check_action():

    # Get Data from the Request
    data = request.get_json()

    # Check Data
    if not data or 'name' not in data or 'ID' not in data or 'type' not in data or 'info' not in data:
        return jsonify({'status': 'error', 'message': 'Invalid input'}), 400

    # Check Command Type
    if not data['type'] in command_list.getCommandTypes():
        return jsonify({'status': 'error', 'message': 'Unknown Command Type'}), 401

    if data['type'] in [NULL, ROS, DEFAULT]:

        # Return Feasibility True
        return jsonify({'status': 'success', 'feasible': True}), 200

    # Check Movement Command Feasibility
    if data['type'] == MOVE:

        # Available Locations
        available_locations = {
            'table': 0,
            'bench': 1,
            'pickup area': 2,
            'park': 3,
        }

        # Check Arguments
        if not all([key in data for key in ['direction', 'distance', 'measure']]) or not 'location' in data:
            return jsonify({'status': 'error', 'message': 'Invalid input'}), 400

        # Create Movement Command
        if 'location' in data: command = MoveCommand(data['name'], data['ID'], data['info'], location=data['location'])
        else: command = MoveCommand(data['name'], data['ID'], data['info'], data['distance'], data['direction'], data['measure'])

        # TODO: Check Feasibility -> Add Some Logic Here
        # Check Available Locations...
        return jsonify({'status': 'success', 'feasible': True}), 200

    # Check Stop Command Feasibility
    elif data['type'] == STOP:

        # Create Stop Command
        command = Command(data['name'], data['ID'], data['type'], data['info'])

        # TODO: Check Feasibility -> Add Some Logic Here
        return jsonify({'status': 'success', 'feasible': True}), 200

    # Check Pick Command Feasibility
    elif data['type'] == PICK:

        available_objects = {
            'scissors': 0,
            'box': 1,
            'hammer': 2,
            'screwdriver': 3
        }

        # Available Locations
        available_locations = {
            'table': 0,
            'bench': 1,
            'pickup area': 2,
            'park': 3,
        }

        # Check Arguments
        if not all([key in data for key in ['object_name', 'location']]):
            return jsonify({'status': 'error', 'message': 'Invalid input'}), 400

        # Create Pick Command
        command = PickCommand(data['name'], data['ID'], data['info'], data['object_name'], data['location'])

        # TODO: Check Feasibility -> Add Some Logic Here
        return jsonify({'status': 'success', 'feasible': True}), 200

    # Check Move Object Command Feasibility
    elif data['type'] == MOVE_OBJECT:

        available_objects = {
            'scissors': 0,
            'box': 1,
            'hammer': 2,
            'screwdriver': 3
        }

        # Available Locations
        available_locations = {
            'table': 0,
            'bench': 1,
            'pickup area': 2,
            'park': 3,
        }

        # Check Arguments
        if not all([key in data for key in ['object_name', 'from_location', 'to_location']]):
            return jsonify({'status': 'error', 'message': 'Invalid input'}), 400

        # Create Move Object Command
        command = MoveObjectCommand(data['name'], data['ID'], data['info'], data['object_name'], data['from_location'], data['to_location'])

        # TODO: Check Feasibility -> Add Some Logic Here
        return jsonify({'status': 'success', 'feasible': True}), 200

    # Check Execute Task Command Feasibility
    elif data['type'] == EXECUTE_TASK:

        available_tasks = {
            'clean': 0,
            'pick': 1,
        }

        # Check Arguments
        if not 'task_name' in data:
            return jsonify({'status': 'error', 'message': 'Invalid input'}), 400

        # Create Execute Task Command
        command = ExecuteTaskCommand(data['name'], data['ID'], data['type'], data['info'], data['task_name'])

        # TODO: Check Feasibility -> Add Some Logic Here
        return jsonify({'status': 'success', 'feasible': True}), 200

if __name__ == '__main__':

    # Run the Flask App
    app.run(debug=True, port=5000)
