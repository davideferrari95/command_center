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
        self.default_labels = ['Name:', 'ID:', 'Type:', 'Info:', 'Status:', 'Command Info:']
        self.command_labels = ['Direction:', 'Distance:', 'Measure:', 'Location:', 'Object Name:', 'Object Location:', 'From Location:', 'To Location:', 'Task Name:']
        label_texts = self.default_labels + self.command_labels

        for text in label_texts:

            # Create Default Labels
            if text in self.default_labels:

                # Font and Background Color - Dark and Light Grey Texts
                label = Label(self.frame, text=text, bg='#F0F0F0', font=('Arial', 16, 'bold'), fg='#333')
                self.labels[text] = Label(self.frame, text='Unknown'.ljust(10), bg='#F0F0F0', font=('Arial', 16), fg='#666')

            # Create Command Labels
            elif text in self.command_labels:

                # Font and Background Color - Dark and Light Grey Texts
                label = Label(self.frame, text=text, bg='#F0F0F0', font=('Arial', 12, 'bold'), fg='#333')
                self.labels[text] = Label(self.frame, text='Unknown', bg='#F0F0F0', font=('Arial', 12), fg='#666')

            # Grid - Forget Layout
            label.grid_forget()
            self.labels[text].grid_forget()

        # Create Initial Static Labels
        Label(self.frame, text='Name:', bg='#F0F0F0', font=('Arial', 16, 'bold'), fg='#333').grid(row=0, column=0, sticky='w', padx=10, pady=10)
        self.labels['Name:'].grid(row=0, column=1, sticky='w', padx=10, pady=10)
        Label(self.frame, text='ID:', bg='#F0F0F0', font=('Arial', 16, 'bold'), fg='#333').grid(row=0, column=2, sticky='w', padx=10, pady=10)
        self.labels['ID:'].grid(row=0, column=3, sticky='w', padx=10, pady=10)
        Label(self.frame, text='Type:', bg='#F0F0F0', font=('Arial', 16, 'bold'), fg='#333').grid(row=0, column=4, sticky='w', padx=10, pady=10)
        self.labels['Type:'].grid(row=0, column=5, sticky='w', padx=10, pady=10)
        Label(self.frame, text='Info:', bg='#F0F0F0', font=('Arial', 16, 'bold'), fg='#333').grid(row=1, column=0, sticky='w', padx=10, pady=10)
        self.labels['Info:'].grid(row=1, column=1, sticky='w', padx=10, pady=10)
        Label(self.frame, text='Status:', bg='#F0F0F0', font=('Arial', 16, 'bold'), fg='#333').grid(row=6, column=0, sticky='w', padx=10, pady=10)
        self.labels['Status:'].grid(row=6, column=1, sticky='w', padx=10, pady=10)
        Label(self.frame, text='Command Info:', bg='#F0F0F0', font=('Arial', 16, 'bold'), fg='#333').grid(row=7, column=0, sticky='w', padx=10, pady=10)
        self.labels['Command Info:'].grid(row=7, column=1, sticky='w', padx=10, pady=10)

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

    def command_callback(self, msg:MultimessageCommand):

        """ Command Callback Function """

        # Hide Command Labels
        self.hide_command_labels()

        # Get Command ID
        command_name, command_id, command_type = msg.name, msg.id, msg.type
        success, error_message = msg.success, msg.error_message
        command = command_list.get_command_by_id(command_id)

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
                    self.labels['Location:'].grid(row=3, column=1, sticky='w')

                else:

                    # Move Command Information
                    self.labels['Direction:'].config(text=command.getDirection(), fg='#007BFF')
                    self.labels['Distance:'].config(text=command.getDistance(), fg='#007BFF')
                    self.labels['Measure:'].config(text=command.getMeasure(), fg='#007BFF')
                    self.labels['Direction:'].grid(row=3, column=1, sticky='w')
                    self.labels['Distance:'].grid(row=3, column=3, sticky='w')
                    self.labels['Measure:'].grid(row=3, column=5, sticky='w')

            # Pick Command Information
            elif isinstance(command, PickCommand):

                self.labels['Object Name:'].config(text=command.getObjectName(), fg='#007BFF')
                self.labels['Location:'].config(text=command.getLocation(), fg='#007BFF')
                self.labels['Object Name:'].grid(row=3, column=1, sticky='w')
                self.labels['Location:'].grid(row=3, column=3, sticky='w')

            # Move Object Command Information
            elif isinstance(command, MoveObjectCommand):

                self.labels['Object Name:'].config(text=command.getObjectName(), fg='#007BFF')
                self.labels['From Location:'].config(text=command.getFromLocation(), fg='#007BFF')
                self.labels['To Location:'].config(text=command.getToLocation(), fg='#007BFF')
                self.labels['Object Name:'].grid(row=3, column=1, sticky='w')
                self.labels['From Location:'].grid(row=3, column=3, sticky='w')
                self.labels['To Location:'].grid(row=3, column=5, sticky='w')

            # Execute Task Command Information
            elif isinstance(command, ExecuteTaskCommand):

                self.labels['Task Name:'].config(text=command.getTaskName(), fg='#007BFF')
                self.labels['Task Name:'].grid(row=3, column=1, sticky='w')

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
