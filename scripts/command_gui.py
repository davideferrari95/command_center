#!/usr/bin/env python

import rospy, rospkg, sys
from typing import Dict
from tkinter import Tk, Label, Frame

# Import ROS Messages
from std_msgs.msg import Int32

# Import Parent Folders
sys.path.append(f'{rospkg.RosPack().get_path("alexa_conversation")}/AzureFunctions')

# Import Command List
from SkillBackend.Utils.command_list  import command_list

class CommandGUI():

    def __init__(self):

        # Initialize Tkinter
        self.root = Tk()
        self.root.title('Command GUI')

        # Set Background Color
        self.root.configure(bg='#F0F0F0')

        # Set Window Size - 720x480 pixels, positioned at 300,150
        # self.root.geometry('720x480+300+150')
        self.root.geometry('720x480')

        # Create Frame and Pack
        self.frame = Frame(self.root, bg='#F0F0F0')
        self.frame.pack(padx=20, pady=20)

        # Labels
        self.labels : Dict[str, Label] = {}
        label_texts = ['Name:', 'ID:', 'Type:', 'Info:']

        for i, text in enumerate(label_texts):

            # Font and Background Color - Dark and Light Grey Texts
            label = Label(self.frame, text=text, bg='#F0F0F0', font=('Arial', 16, 'bold'), fg='#333')
            self.labels[text] = Label(self.frame, text='Unknown', bg='#F0F0F0', font=('Arial', 16), fg='#666')

            # Grid layout
            label.grid(row=i, column=0, sticky='w', padx=10, pady=5)
            self.labels[text].grid(row=i, column=1, sticky='w', padx=10, pady=5)

        # Initialize ROS Node
        rospy.init_node('command_gui', anonymous=True)
        rospy.Subscriber('command_topic', Int32, self.command_callback)
        
    def command_callback(self, msg:Int32):

        # Get Command Information
        command_id = msg.data
        command = command_list.get_command_by_id(command_id)

        if command:

            # Update Command Information - Blue Text
            self.labels['Name:'].config(text=command.getName(), fg='#007BFF')
            self.labels['ID:'  ].config(text=command.getID())
            self.labels['Type:'].config(text=command.getType(), fg='#007BFF')
            self.labels['Info:'].config(text=command.getInfo(), fg='#007BFF')

        else:

            # Set Unknown Command Information - Red Text
            self.labels['Name:'].config(text='Unknown', fg='#DC3545')
            self.labels['ID:'  ].config(text=command_id)
            self.labels['Type:'].config(text='Unknown', fg='#DC3545')
            self.labels['Info:'].config(text='Unknown', fg='#DC3545')

    def run(self):

        # Run Tkinter Main Loop
        self.root.mainloop()

if __name__ == '__main__':

    # Command GUI
    gui = CommandGUI()
    gui.run()
