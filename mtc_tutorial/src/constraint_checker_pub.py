#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from task_planner_interfaces.srv import PlanningAction  # Replace with your package name
from std_msgs.msg import String

import os
import base64
from openai import OpenAI

messages = [
    {'role': 'system', 'content': """
You are a constraint checker for a robot manipulator and an excellent interpreter of human instructions.
Given an image, you analyze it and check if the given constraint is being met. 

If the constraint is met, you answer 'Yes'. If the constraint is not met, you answer 'No'.

You must not add redundant information or characters in the response.
"""},
]


#while True:

# task
constraint1 = 'Is the gripper grasping the cylindrical?'
if message:
    messages.append(
        {"role": "user", "content": message},
    )
    chat = openai.chat.completions.create(
        model="gpt-4o", messages=messages
    )

reply = chat.choices[0].message.content
print(f"Constraint Checker: \n{reply}")
messages.append({"role": "assistant", "content": reply})
action_list = []
action_list = reply.split(",")



class ConstraintCheckerServer(Node):

    def __init__(self):
        super().__init__('constraint_checker_publisher')
        # self.srv = self.create_service(PlanningAction, 'task_planner_service', self.task_planner_callback)

        self.publisher_ = self.create_publisher(String, 'constraints_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    
    def timer_callback(self):
        msg = String()
        msg.data = 'yes, no'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = ConstraintCheckerServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()




