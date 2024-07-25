#!/usr/bin/env python3

import sys, rospkg
from flask import Flask, request, jsonify
from termcolor import colored
from typing import Union

# Import Parent Folders
sys.path.append(f'{rospkg.RosPack().get_path("alexa_conversation")}/AzureFunctions')

# Import Commands
from SkillBackend.Utils.command_list  import *

# Command Union Type
CommandUnion = Union[Command, PickCommand, MoveCommand, MoveObjectCommand, ExecuteTaskCommand]

# Create the Flask App
app = Flask(__name__)

# Available Objects
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

# Object Default Location
object_default_location = {
    'scissors': 'table',
    'hammer': 'bench',
    'box': 'pickup area',
    'screwdriver': 'bench'
}

def return_function(command:CommandUnion, success:bool=False, json_status:int=400,  error_message:str='') -> jsonify:

    """ Return Function to Send Command to ROS and JSON Response """

    # Print Command Status
    if success: print(colored('\nSUCCESS: ', 'green'), f'Feasible Command - {command.getName()}\n')
    else: print(colored('\nERROR: ', 'red'), f'Infeasible Command - {command.getName()}\n', f'{error_message}\n')

    # Return JSON Response
    if success: return jsonify({'status': 'success', 'feasible': True}), json_status
    else: return jsonify({'status': 'error', 'message': error_message}), json_status

# Check Action Route
@app.route('/check_action', methods=['POST'])
def check_action():

    """ Check Action Route """

    # Get Data from the Request
    data = request.get_json()
    print(colored('\nReceived Data: \n', 'blue'))
    for d in data: print(f'\t{d}: {data[d]}')

    # Check Data
    if not data or 'name' not in data or 'ID' not in data or 'type' not in data or 'info' not in data:

        # Unknown Command Creation
        command = Command('Unknown', 'Unknown', 'Unknown', 'Unknown') if not data else \
                  Command('Unknown' if not 'name' in data else data['name'], 'Unknown' if not 'ID' in data else data['ID'],
                          'Unknown' if not 'type' in data else data['type'], 'Unknown' if not 'info' in data else data['info'])

        # Return Function
        return return_function(command, False, 400, 'Invalid Input - Missing Data or Missing ID, Type, Name, or Info')

    # Check Command Type
    if not data['type'] in command_list.getCommandTypes():

        # Return Unknown Command Type
        return return_function(Command(data['name'], data['ID'], data['type'], data['info']), False, 401, f'Unknown Command Type: {data["type"]}')

    if data['type'] in [NULL, ROS, DEFAULT]:

        # Return Feasibility True
        return return_function(Command(data['name'], data['ID'], data['type'], data['info']), True, 200)

    # Check Movement Command Feasibility
    if data['type'] == MOVE:

        # Check Arguments
        if not all([key in data for key in ['direction', 'distance', 'measure']]) and not 'location' in data:
            return return_function(MoveCommand(data['name'], data['ID'], data['info']), False, 400, 'Invalid Input - Missing Direction, Distance, Measure, or Location')

        # Check Location
        if 'location' in data:

            # Create GOTO Command
            command = MoveCommand(data['name'], data['ID'], data['info'], location=data['location'])

            # TODO: Check Feasibility Logic Here

            # Check Available Locations...
            if not data['location'] in available_locations:
                return return_function(command, False, 400, f'Invalid Input - Unknown Location {data["location"]}')

            # Return Feasibility True
            return return_function(command, True, 200)

        else:

            # Create Move Command
            command = MoveCommand(data['name'], data['ID'], data['info'], data['distance'], data['direction'], data['measure'])

            # TODO: Check Feasibility Logic Here

            # Check Direction
            if not data['direction'] in ['forward', 'backward', 'left', 'right']:
                return return_function(command, False, 400, f'Invalid Input - Unknown Direction {data["direction"]}')

            # Check Distance
            if not isinstance(data['distance'], (int, float)) or data['distance'] < 0:
                return return_function(command, False, 400, f'Invalid Input - Unknown Distance {data["distance"]}')

            # Check Measure
            if not data['measure'] in ['meters', 'centimeters']:
                return return_function(command, False, 400, f'Invalid Input - Unknown Measure {data["measure"]}')

            # Check Measure and Distance - Centimeters < 20, Meters < 0.2
            if (data['measure'] == 'centimeters' and data['distance'] < 20.0) or (data['measure'] == 'meters' and data['distance'] < 0.2):
                return return_function(command, False, 400, f'Distance too Short - I can\'t move Distance {data["distance"]} {data["measure"]}')

            # Check Measure and Distance - Centimeters > 500, Meters > 5
            if (data['measure'] == 'centimeters' and data['distance'] > 500.0) or (data['measure'] == 'meters' and data['distance'] > 5.0):
                return return_function(command, False, 400, f'Distance too Long - I can\'t move Distance {data["distance"]} {data["measure"]}')

            # Return Feasibility True
            return return_function(command, True, 200)

    # Check Stop Command Feasibility
    elif data['type'] == STOP:

        # Create Stop Command
        command = Command(data['name'], data['ID'], data['type'], data['info'])

        # TODO: Check Feasibility Logic Here

        # Return Feasibility True
        return return_function(command, True, 200)

    # Check Pick Command Feasibility
    elif data['type'] == PICK:

        # Check Arguments
        if not all([key in data for key in ['object_name', 'location']]):
            return return_function(PickCommand(data['name'], data['ID'], data['info']), False, 400, 'Invalid Input - Missing Object Name or Location')

        # Check 'Null' Location
        if data['location'] in ['null','chimera','']:
            data['location'] = 'Unknown' if not data['object_name'] in object_default_location else object_default_location[data['object_name']]

        # Create Pick Command
        command = PickCommand(data['name'], data['ID'], data['info'], data['object_name'], data['location'])

        # TODO: Check Feasibility Logic Here

        # Check Available Objects...
        if not data['object_name'] in available_objects:
            return return_function(command, False, 400, f'Invalid Input - Unknown Object {data["object_name"]}')

        # Check Available Locations...
        if not data['location'] in available_locations:
            return return_function(command, False, 400, f'Invalid Input - Unknown Location {data["location"]}')

        # Return Feasibility True
        return return_function(command, True, 200)

    # Check Move Object Command Feasibility
    elif data['type'] == MOVE_OBJECT:

        # Check Arguments
        if not all([key in data for key in ['object_name', 'from_location', 'to_location']]):
            return return_function(MoveObjectCommand(data['name'], data['ID'], data['info']), False, 400, 'Invalid Input - Missing Object Name, From Location, or To Location')

        # Check 'Null' From Location
        if data['from_location'] in ['null','chimera','']:
            data['from_location'] = 'Unknown' if not data['object_name'] in object_default_location else object_default_location[data['object_name']]

        # Create Move Object Command
        command = MoveObjectCommand(data['name'], data['ID'], data['info'], data['object_name'], data['from_location'], data['to_location'])

        # TODO: Check Feasibility Logic Here

        # Check Available Objects...
        if not data['object_name'] in available_objects:
            return return_function(command, False, 400, f'Invalid Input - Unknown Object {data["object_name"]}')

        # Check Available Locations...
        if not data['from_location'] in available_locations:
            return return_function(command, False, 400, f'Invalid Input - Unknown From Location {data["from_location"]}')
        elif not data['to_location'] in available_locations:
            return return_function(command, False, 400, f'Invalid Input - Unknown To Location {data["to_location"]}')

        # Return Feasibility True
        return return_function(command, True, 200)

    # Check Execute Task Command Feasibility
    elif data['type'] == EXECUTE_TASK:

        available_tasks = {
            'clean': 0,
            'pick': 1,
        }

        # Check Arguments
        if not 'task_name' in data:
            return return_function(ExecuteTaskCommand(data['name'], data['ID'], data['info']), False, 400, 'Invalid Input - Missing Task Name')

        # Create Execute Task Command
        command = ExecuteTaskCommand(data['name'], data['ID'], data['info'], data['task_name'])

        # TODO: Check Feasibility Logic Here

        # Check Available Tasks...
        if not data['task_name'] in available_tasks:
            return return_function(command, False, 400, f'Invalid Input - Unknown Task {data["task_name"]}')

        # Return Feasibility True
        return return_function(command, True, 200)

if __name__ == '__main__':

    # Run the Flask App
    app.run(debug=True, port=5000)
