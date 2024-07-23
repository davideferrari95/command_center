#!/usr/bin/env python

import rospy, rospkg, sys
from typing import Dict
from tkinter import Tk, Label, Frame

# Import ROS Command Messages
from alexa_conversation.msg import MultimessageCommand, PickCommand, MoveCommand, MoveObjectCommand, ExecuteTaskCommand

# Import Parent Folders
sys.path.append(f'{rospkg.RosPack().get_path("alexa_conversation")}/AzureFunctions')

# Import Command List
from SkillBackend.Utils.command_list  import *

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
        self.labels : Dict[str, Label] = {}
        default_labels = ['Name:', 'ID:', 'Type:', 'Info:']
        command_labels = ['Direction', 'Distance:', 'Measure:', 'Location:', 'Object Name:', 'Object Location', 'From Location', 'To Location', 'Task Name', 'Status:', 'Command Info:']
        label_texts = default_labels + command_labels

        # for i, text in enumerate(label_texts):

        #     # Font and Background Color - Dark and Light Grey Texts
        #     label = Label(self.frame, text=text, bg='#F0F0F0', font=('Arial', 16, 'bold'), fg='#333')
        #     self.labels[text] = Label(self.frame, text='Unknown', bg='#F0F0F0', font=('Arial', 16), fg='#666')

        #     # Grid layout
        #     label.grid(row=i, column=0, sticky='w', padx=10, pady=5)
        #     self.labels[text].grid(row=i, column=1, sticky='w', padx=10, pady=5)

        # Creating Labels
        for text in label_texts:

            if text in ['Name:', 'ID:', 'Type:', 'Info:', 'Status:', 'Command Info:']:

                # Font and Background Color - Dark and Light Grey Texts
                label = Label(self.frame, text=text, bg='#F0F0F0', font=('Arial', 16, 'bold'), fg='#333')
                self.labels[text] = Label(self.frame, text='Unknown', bg='#F0F0F0', font=('Arial', 16), fg='#666')

                # Grid layout
                if text == 'Name:':

                    # Name - Row 0, Column 0,1
                    label.grid(row=0, column=0, sticky='w', padx=10, pady=5)
                    self.labels[text].grid(row=0, column=1, sticky='w', padx=50, pady=10)

                elif text == 'ID:':

                    # ID - Row 0, Column 2,3
                    label.grid(row=0, column=2, sticky='w', padx=10, pady=5)
                    self.labels[text].grid(row=0, column=3, sticky='w', padx=50, pady=10)

                elif text == 'Type:':

                    # Type - Row 0, Column 4,5
                    label.grid(row=0, column=4, sticky='w', padx=10, pady=5)
                    self.labels[text].grid(row=0, column=5, sticky='w', padx=50, pady=10)

                elif text == 'Info:':

                    # Info - Row 1, Column 0,1
                    label.grid(row=1, column=0, sticky='w', padx=10, pady=5)
                    self.labels[text].grid(row=1, column=1, sticky='w', padx=50, pady=5)

                elif text == 'Status:':

                    # Status - Row 6, Column 0,1
                    label.grid(row=6, column=0, sticky='w', padx=10, pady=5)
                    self.labels[text].grid(row=6, column=1, sticky='w', padx=50, pady=5)

                elif text == 'Command Info:':

                    # Command Info - Row 7, Column 0,1
                    label.grid(row=7, column=0, sticky='w', padx=10, pady=5)
                    self.labels[text].grid(row=7, column=1, sticky='w', padx=50, pady=5)

            else:

                # Font and Background Color - Dark and Light Grey Texts
                label = Label(self.frame, text=text, bg='#F0F0F0', font=('Arial', 12, 'bold'), fg='#333')
                self.labels[text] = Label(self.frame, text='Unknown', bg='#F0F0F0', font=('Arial', 12), fg='#666')

                # Grid layout
                if text == 'Direction:':

                    # Direction - Row 3, Column 0,1
                    label.grid(row=3, column=0, sticky='w', padx=10, pady=5)
                    self.labels[text].grid(row=3, column=1, sticky='w', padx=50, pady=5)

                elif text == 'Distance:':

                    # Distance - Row 3, Column 2,3
                    label.grid(row=3, column=2, sticky='w', padx=10, pady=5)
                    self.labels[text].grid(row=3, column=3, sticky='w', padx=50, pady=5)

                elif text == 'Measure:':

                    # Measure - Row 3, Column 4,5
                    label.grid(row=3, column=4, sticky='w', padx=10, pady=5)
                    self.labels[text].grid(row=3, column=5, sticky='w', padx=50, pady=5)

                elif text == 'Location:':

                    # Location - Row 4, Column 0,1
                    label.grid(row=4, column=0, sticky='w', padx=10, pady=5)
                    self.labels[text].grid(row=4, column=1, sticky='w', padx=50, pady=5)                    

                elif text == 'Object Name:':

                    # Object Name - Row 3, Column 0,1
                    label.grid(row=3, column=0, sticky='w', padx=10, pady=5)
                    self.labels[text].grid(row=3, column=1, sticky='w', padx=50, pady=5)

                elif text == 'Object Location:':

                    # Object Location - Row 3, Column 2,3
                    label.grid(row=3, column=2, sticky='w', padx=10, pady=5)
                    self.labels[text].grid(row=3, column=3, sticky='w', padx=50, pady=5)

                elif text == 'From Location:':

                    # From Location - Row 3, Column 2,3
                    label.grid(row=3, column=2, sticky='w', padx=10, pady=5)
                    self.labels[text].grid(row=3, column=3, sticky='w', padx=50, pady=5)

                elif text == 'To Location:':

                    # To Location - Row 3, Column 4,5
                    label.grid(row=3, column=4, sticky='w', padx=10, pady=5)
                    self.labels[text].grid(row=3, column=5, sticky='w', padx=50, pady=5)

                elif text == 'Task Name:':

                    # Task Name - Row 3, Column 0,1
                    label.grid(row=3, column=0, sticky='w', padx=10, pady=5)
                    self.labels[text].grid(row=3, column=1, sticky='w', padx=50, pady=5)

        # Creating Spacing in Between Sections
        label = Label(self.frame, text=' ', bg='#F0F0F0', font=('Arial', 32, 'bold'), fg='#333')
        label.grid(row=2, column=0, sticky='w', padx=10, pady=5)
        label = Label(self.frame, text=' ', bg='#F0F0F0', font=('Arial', 32, 'bold'), fg='#333')
        label.grid(row=5, column=0, sticky='w', padx=10, pady=5)

        # Initialize ROS Node
        rospy.init_node('command_gui', anonymous=True)
        rospy.Subscriber('alexa/command', MultimessageCommand, self.command_callback)

    def command_callback(self, msg:MultimessageCommand):

        # Get Command ID
        command_name, command_id, command_type = msg.name, msg.id, msg.type
        success, error_message = msg.success, msg.error_message
        command = command_list.get_command_by_id(id)

        # Hide all optional fields initially
        for text in ['Distance:', 'Location:', 'Measure:', 'Object Name:']:
            self.labels[text].config(text='N/A', fg='#666')

        if command:

            # Update Command Information - Blue Text
            self.labels['Name:'].config(text=command.getName(), fg='#007BFF')
            self.labels['ID:'  ].config(text=command.getID())
            self.labels['Type:'].config(text=command.getType(), fg='#007BFF')
            self.labels['Info:'].config(text=command.getInfo(), fg='#007BFF')

            # Move Command Information
            if isinstance(command, MoveCommand):

                if command.getLocation() not in ['null', 'chimera']:

                    # GOTO Command Information
                    self.labels['Location:'].config(text=command.getLocation(), fg='#007BFF')

                else:

                    # Move Command Information
                    self.labels['Direction:'].config(text=command.getDirection(), fg='#007BFF')
                    self.labels['Distance:'].config(text=command.getDistance(), fg='#007BFF')
                    self.labels['Measure:'].config(text=command.getMeasure(), fg='#007BFF')

            # Pick Command Information
            elif isinstance(command, PickCommand):

                self.labels['Object Name:'].config(text=command.getObjectName(), fg='#007BFF')
                self.labels['Location:'].config(text=command.getLocation(), fg='#007BFF')

            # Move Object Command Information
            elif isinstance(command, MoveObjectCommand):

                self.labels['Object Name:'].config(text=command.getObjectName(), fg='#007BFF')
                self.labels['From Location:'].config(text=command.getFromLocation(), fg='#007BFF')
                self.labels['To Location:'].config(text=command.getToLocation(), fg='#007BFF')

            # Execute Task Command Information
            elif isinstance(command, ExecuteTaskCommand):

                self.labels['Task Name'].config(text=command.getTaskName(), fg='#007BFF')

            else:

                # self.labels['Direction:'].config(text='N/A', fg='#666')
                self.labels['Distance:'].config(text='N/A', fg='#666')
                self.labels['Measure:'].config(text='N/A', fg='#666')
                self.labels['Location:'].config(text='N/A', fg='#666')
                self.labels['Object Name:'].config(text='N/A', fg='#666')
                self.labels['From Location:'].config(text='N/A', fg='#666')
                self.labels['To Location:'].config(text='N/A', fg='#666')
                self.labels['Task Name:'].config(text='N/A', fg='#666')

            self.labels['Status:'].config(text='SUCCESS', fg='#007BFF')
            self.labels['Command Info:'].config(text='Command executed successfully', fg='#007BFF')

        else:

            # Set Unknown Command Information - Red Text
            self.labels['Name:'].config(text='Unknown', fg='#DC3545')
            self.labels['ID:'  ].config(text=command_id)
            self.labels['Type:'].config(text='Unknown', fg='#DC3545')
            self.labels['Info:'].config(text='Unknown', fg='#DC3545')

            # Command Status and Info
            self.labels['Status:'].config(text='FAIL', fg='#DC3545')
            self.labels['Command Info:'].config(text='Command not recognized', fg='#DC3545')

    def run(self):

        # Run Tkinter Main Loop
        self.root.mainloop()

if __name__ == '__main__':

    # Command GUI
    gui = CommandGUI()
    gui.run()
