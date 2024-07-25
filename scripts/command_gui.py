#!/usr/bin/env python

import rospy, rospkg, sys
from typing import Dict, Union, Optional
from tkinter import Tk, Label, Frame

# Import ROS Command Messages
from alexa_conversation.msg import MultimessageCommand, PickCommand, MoveCommand, MoveObjectCommand, ExecuteTaskCommand

# Import Parent Folders
sys.path.append(f'{rospkg.RosPack().get_path("alexa_conversation")}/AzureFunctions')

# Import Command List
from SkillBackend.Utils.command_list  import *

# Command Union Type
CommandUnion = Union[Command, PickCommand, MoveCommand, MoveObjectCommand, ExecuteTaskCommand]

class CommandGUI():

    def __init__(self):

        # Initialize Tkinter
        self.root = Tk()
        self.root.title('Command GUI')

        # Set Background Color
        self.root.configure(bg='#F0F0F0')

        # Set Window Size - 720x480 pixels, positioned at 300,150
        # self.root.geometry('720x480+300+150')
        # self.root.geometry('720x480')
        self.root.geometry('1280x720')
        self.root.minsize(720, 480)
        self.root.maxsize(1920, 1080)
        self.root.resizable(True, True)

        # Create Frame and Pack
        self.frame = Frame(self.root, bg='#F0F0F0')
        self.frame.pack(padx=20, pady=20)

        # Labels
        self.labels: Dict[str, Label] = {}
        self.labels_data: Dict[str, Label] = {}
        self.default_labels = ['Name:', 'ID:', 'Type:', 'Info:', 'Status:', 'Command Info:']
        self.command_labels = ['Direction:', 'Distance:', 'Measure:', 'Location:', 'Object Name:', 'Object Location:', 'From Location:', 'To Location:', 'Task Name:']
        label_texts = self.default_labels + self.command_labels

        for text in label_texts:

            # Create Default Labels Data - Name, ID, Type, Info, Status, Command Info
            if text in self.default_labels: self.labels_data[text] = Label(self.frame, text='Unknown'.ljust(10), bg='#F0F0F0', font=('Arial', 16), fg='#666')

            # Create Command Labels Data - Direction, Distance, Measure, Location, Object Name, Object Location, From Location, To Location, Task Name
            elif text in self.command_labels:

                # Font and Background Color - Dark and Light Grey Texts
                self.labels[text] = Label(self.frame, text=text, bg='#F0F0F0', font=('Arial', 16, 'bold'), fg='#333')
                self.labels_data[text] = Label(self.frame, text='Unknown', bg='#F0F0F0', font=('Arial', 16), fg='#666')

                # Grid - Forget Layout Labels
                self.labels[text].grid_forget()

            # Grid - Forget Layout Labels Data
            self.labels_data[text].grid_forget()

        # Create Initial Static Labels - Name, ID, Type, Info
        Label(self.frame, text='Name:', bg='#F0F0F0', font=('Arial', 16, 'bold'), fg='#333').grid(row=0, column=0, sticky='w', padx=10, pady=10)
        self.labels_data['Name:'].grid(row=0, column=1, sticky='w', padx=10, pady=10)
        Label(self.frame, text='ID:', bg='#F0F0F0', font=('Arial', 16, 'bold'), fg='#333').grid(row=0, column=2, sticky='w', padx=10, pady=10)
        self.labels_data['ID:'].grid(row=0, column=3, sticky='w', padx=20, pady=10)
        Label(self.frame, text='Type:', bg='#F0F0F0', font=('Arial', 16, 'bold'), fg='#333').grid(row=0, column=4, sticky='w', padx=10, pady=10)
        self.labels_data['Type:'].grid(row=0, column=5, sticky='w', padx=10, pady=10)
        Label(self.frame, text='Info:', bg='#F0F0F0', font=('Arial', 16, 'bold'), fg='#333').grid(row=1, column=0, sticky='w', padx=10, pady=10)
        self.labels_data['Info:'].grid(row=1, column=1, columnspan=5, sticky='w', padx=10, pady=10)

        # Create Initial Static Labels - Status, Command Info
        Label(self.frame, text='Status:', bg='#F0F0F0', font=('Arial', 16, 'bold'), fg='#333').grid(row=6, column=0, columnspan=2, sticky='w', padx=10, pady=10)
        self.labels_data['Status:'].grid(row=6, column=2, columnspan=4, sticky='w', padx=10, pady=10)
        Label(self.frame, text='Command Info:', bg='#F0F0F0', font=('Arial', 16, 'bold'), fg='#333').grid(row=7, column=0, columnspan=2, sticky='w', padx=10, pady=10)
        self.labels_data['Command Info:'].grid(row=7, column=2, columnspan=4, sticky='w', padx=10, pady=10)

        # Creating Spacing in Between Sections
        label = Label(self.frame, text=' ', bg='#F0F0F0', font=('Arial', 32, 'bold'), fg='#333')
        label.grid(row=2, column=0, sticky='w')
        label = Label(self.frame, text=' ', bg='#F0F0F0', font=('Arial', 32, 'bold'), fg='#333')
        label.grid(row=5, column=0, sticky='w')

        rospy.init_node('command_gui', anonymous=True)
        rospy.Subscriber('alexa/command', MultimessageCommand, self.command_callback)

    def hide_command_labels(self):

        """ Hide Command Labels Helper Function """

        # Hide Command Labels
        for label in self.command_labels:
            self.labels[label].grid_forget()
            self.labels_data[label].grid_forget()

    def command_callback(self, msg:MultimessageCommand):

        """ Command Callback Function """

        # Hide Command Labels
        self.hide_command_labels()

        # Get Command Name, ID, and Type
        command_name, command_id, command_type = msg.name, msg.id, msg.type
        success, error_message = msg.success, msg.error_message

        # Get Command from Command List
        command:Optional[CommandUnion] = command_list.get_command_by_id(command_id)

        if command is not None:

            # Fill Command Information
            if command_type in [NULL, ROS, DEFAULT, STOP]: pass
            elif command_type == MOVE: command.fill(msg.move_command)
            elif command_type == PICK: command.fill(msg.pick_command)
            elif command_type == MOVE_OBJECT: command.fill(msg.move_object_command)
            elif command_type == EXECUTE_TASK: command.fill(msg.execute_task_command)

            # Update Command Information - Blue Text
            self.labels_data['Name:'].config(text=command.getName(), fg='#007BFF')
            self.labels_data['ID:'  ].config(text=command.getID())
            self.labels_data['Type:'].config(text=command.getType(), fg='#007BFF')
            self.labels_data['Info:'].config(text=command.getInfo(), fg='#007BFF')

            # Move Command Information
            if isinstance(command, MoveCommand):

                if command.getLocation() not in ['null', 'chimera', '']:

                    # GOTO Command Information
                    self.labels['Location:'].grid(row=3, column=0, columnspan=2, sticky='w')
                    self.labels_data['Location:'].config(text=command.getLocation().capitalize(), fg='#FFBF00')
                    self.labels_data['Location:'].grid(row=3, column=2, columnspan=4, sticky='w')

                else:

                    # Move Command Information
                    self.labels['Direction:'].grid(row=3, column=0, sticky='w')
                    self.labels['Distance:'].grid(row=3, column=2, sticky='w')
                    # self.labels['Measure:'].grid(row=3, column=4, sticky='w')
                    self.labels_data['Direction:'].config(text=command.getDirection().capitalize(), fg='#FFBF00')
                    self.labels_data['Distance:'].config(text=command.getDistance(), fg='#FFBF00')
                    self.labels_data['Measure:'].config(text=command.getMeasure(), fg='#FFBF00')
                    self.labels_data['Direction:'].grid(row=3, column=1, sticky='w')
                    self.labels_data['Distance:'].grid(row=3, column=3, sticky='w', padx=5)
                    self.labels_data['Measure:'].grid(row=3, column=4, sticky='w')

            # Pick Command Information
            elif isinstance(command, PickCommand):

                self.labels['Object Name:'].grid(row=3, column=0, columnspan=2, sticky='w')
                self.labels['Location:'].grid(row=3, column=4, columnspan=2, sticky='w')
                self.labels_data['Object Name:'].config(text=command.getObjectName().capitalize(), fg='#FFBF00')
                self.labels_data['Location:'].config(text=command.getLocation().capitalize(), fg='#FFBF00')
                self.labels_data['Object Name:'].grid(row=3, column=2, sticky='w')
                self.labels_data['Location:'].grid(row=3, column=6, sticky='w')

            # Move Object Command Information
            elif isinstance(command, MoveObjectCommand):

                self.labels['Object Name:'].grid(row=3, column=0, columnspan=2, sticky='w')
                self.labels['From Location:'].grid(row=3, column=3, sticky='w')
                self.labels['From Location:'].config(text='From:', padx=30)
                self.labels['To Location:'].grid(row=3, column=5, sticky='w', padx=10)
                self.labels['To Location:'].config(text='To: ')
                self.labels_data['Object Name:'].config(text=command.getObjectName().capitalize(), fg='#FFBF00')
                self.labels_data['From Location:'].config(text=command.getFromLocation().capitalize(), fg='#FFBF00')
                self.labels_data['To Location:'].config(text=command.getToLocation().capitalize(), fg='#FFBF00')
                self.labels_data['Object Name:'].grid(row=3, column=2, sticky='w')
                self.labels_data['From Location:'].grid(row=3, column=4, sticky='w')
                self.labels_data['To Location:'].grid(row=3, column=6, sticky='w')

            # Execute Task Command Information
            elif isinstance(command, ExecuteTaskCommand):

                self.labels['Task Name:'].grid(row=3, column=0, columnspan=2, sticky='w')
                self.labels_data['Task Name:'].config(text=command.getTaskName().capitalize(), fg='#FFBF00')
                self.labels_data['Task Name:'].grid(row=3, column=2, columnspan=2, sticky='w')

            self.labels_data['Status:'].config(text='SUCCESS' if success else 'FAIL', fg='#34D01F' if success else '#DC3545')
            self.labels_data['Command Info:'].config(text=error_message, fg='#34D01F' if success else '#DC3545')

        else:

            # Set Unknown Command Information - Red Text
            self.labels_data['Name:'].config(text='Unknown', fg='#DC3545')
            self.labels_data['ID:'  ].config(text=command_id)
            self.labels_data['Type:'].config(text='Unknown', fg='#DC3545')
            self.labels_data['Info:'].config(text='Unknown', fg='#DC3545')

            # Command Status and Info
            self.labels_data['Status:'].config(text='FAIL', fg='#DC3545')
            self.labels_data['Command Info:'].config(text='Command not recognized', fg='#DC3545')

    def run(self):

        # Run Tkinter Main Loop
        self.root.mainloop()

if __name__ == '__main__':

    # Command GUI
    gui = CommandGUI()
    gui.run()
