#!/usr/bin/env python
import tkinter as tk
from tkinter import ttk
import rospy
from std_msgs.msg import UInt16, Bool, Int32
from sensor_msgs.msg import JointState

class ServoControlApp:
    def __init__(self, master):
        self.master = master
        self.master.title("Servo Control")
        self.switch_pub = rospy.Publisher('switch_topic', Bool, queue_size=10)
        # ROS Initialization
        rospy.init_node('servo_control')
        self.servo_pub = rospy.Publisher('servo1', UInt16, queue_size=10)
        self.servo2_pub = rospy.Publisher('servo2', UInt16, queue_size=10)
        self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        
        # Create Frame for Servo 1
        self.frame1 = ttk.Frame(master, borderwidth=2, relief="groove")
        self.frame1.pack(padx=10, pady=10, fill=tk.BOTH, expand=True)

        # Create Scale Widget for Servo 1 Angle
        self.angle_label = ttk.Label(self.frame1, text="Servo 1 Angle (0-155 degrees):")
        self.angle_label.pack()
        self.angle_scale = ttk.Scale(self.frame1, from_=0, to=155, length=400 , orient=tk.HORIZONTAL, command=self.update_servo)
        self.angle_scale.set(75)  # Set initial value to 0 degrees
        self.angle_scale.pack()
    
        # Label to display servo 1 angle
        self.servo1_angle_label = ttk.Label(self.frame1, text="Current Angle of Servo 1: 0 degrees")
        self.servo1_angle_label.pack()

        # Create Frame for Servo 2
        self.frame2 = ttk.Frame(master, borderwidth=2, relief="groove")
        self.frame2.pack(padx=10, pady=10, fill=tk.BOTH, expand=True)

        # Create Scale Widget for Servo 2 Angle
        self.angle_label2 = ttk.Label(self.frame2, text="Servo 2 Angle (0-175 degrees):")
        self.angle_label2.pack()
        self.angle_scale2 = ttk.Scale(self.frame2, from_=0, to=175 ,length=400, orient=tk.HORIZONTAL, command=self.update_servo2)
        self.angle_scale2.set(0)  # Set initial value to 0 degrees
        self.angle_scale2.pack()

        # Label to display servo 2 angle
        self.servo2_angle_label = ttk.Label(self.frame2, text="Current Angle of Servo 2: 0 degrees")
        self.servo2_angle_label.pack()

        # Create Reset Button
        self.reset_button = ttk.Button(master, text="Reset", command=self.reset_servos)
        self.reset_button.pack()

        # Create Switch Button
        self.mode_state_var = tk.BooleanVar()
        self.mode_button = tk.Button(self.master, text="Mode", command=self.toggle_mode)
        self.mode_button.pack(pady=10)

        # ROS Subscriber for positionJoint1
        rospy.Subscriber("positionJoint1", Int32, self.update_position_joint1)
        # ROS Subscriber for positionJoint2
        rospy.Subscriber("positionJoint2", Int32, self.update_position_joint2)

    def update_servo(self, angle):
        angle_int = int(float(angle))
        self.servo_pub.publish(angle_int)
        self.servo1_angle_label.config(text="Current Angle of Servo 1: {} degrees".format(angle_int))

    def update_servo2(self, angle):
        angle_int = int(float(angle))
        self.servo2_pub.publish(angle_int)
        self.servo2_angle_label.config(text="Current Angle of Servo 2: {} degrees".format(angle_int))

    def reset_servos(self):
        self.angle_scale.set(75)  # Set motor 1 angle to 75 degrees
        self.angle_scale2.set(0)  # Set motor 2 angle to 0 degrees
        self.update_servo(75)  # Update motor 1 angle display
        self.update_servo2(0)  # Update motor 2 angle display

    def toggle_mode(self):
        mode_state = not self.mode_state_var.get()
        self.send_mode_to_arduino(mode_state)
        self.mode_state_var.set(mode_state)

    def send_mode_to_arduino(self, mode_state):
        rospy.Publisher('mode', Bool, queue_size=10).publish(mode_state)

    def update_position_joint1(self, position):
        if position is None:
            position_val = 0.0  # Set default value if None
        else:
            position_val = position.data
        mapped_position_joint1, _ = self.map_joint_values(position_val, 0.0)  # Mapping position_joint1
        self.servo1_angle_label.config(text="PositionJoint1: {:.2f} degrees".format(mapped_position_joint1))
        self.publish_joint_state(mapped_position_joint1, 0.0)  # Passing mapped_position_joint1 and 0.0 as default value for position_joint2

    def update_position_joint2(self, position):
        if position is None:
            position_val = 0.0  # Set default value if None
        else:
            position_val = position.data
        _, mapped_position_joint2 = self.map_joint_values(0.0, position_val)  # Mapping position_joint2
        self.servo2_angle_label.config(text="PositionJoint2: {:.2f} degrees".format(mapped_position_joint2))
        self.publish_joint_state(0.0, mapped_position_joint2)  # Passing 0.0 as default value for position_joint1 and mapped_position_joint2

    def map_value(self, value, from_min, from_max, to_min, to_max):
        from_range = from_max - from_min
        to_range = to_max - to_min
        scaled_value = float(value - from_min) / float(from_range)
        return to_min + (scaled_value * to_range)

    def map_joint_values(self, position_joint1, position_joint2):
        mapped_position_joint1 = self.map_value(position_joint1, 0, 155, -3.00, 3.00)
        mapped_position_joint2 = self.map_value(position_joint2, 0, 175, -1.50, 1.50)
        return round(mapped_position_joint1, 2), round(mapped_position_joint2, 2)

    def publish_joint_state(self, position_joint1, position_joint2):
        mapped_position_joint1, mapped_position_joint2 = self.map_joint_values(position_joint1, position_joint2)
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = ['joint1', 'joint2']
    
        joint_state_msg.position = [mapped_position_joint1, mapped_position_joint2]
    
        self.joint_state_pub.publish(joint_state_msg)

def main():
    root = tk.Tk()
    app = ServoControlApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()
