#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from action_tutorials_interfaces.srv import PlanningAction  # Replace with your package name

import os
import openai

messages = [
    {'role': 'system', 'content': """
You are a planner for a robot manipulator and an excellent interpreter of human instructions.
Given an instruction, you break it down into a sequence of robot skills.

The available list of skills are as follows:

"grasp"
"ungrasp"
"move to pick" 
"move to place"

Precisely return the following:
1. The correct sequence of skills only to perform the required task. You must not include any extra information in the response other than the sequence of skills.
"""},
]


#while True:

# task
message = 'Task: Pick the object and place it on the table.'
if message:
    messages.append(
        {"role": "user", "content": message},
    )
    chat = openai.chat.completions.create(
        model="gpt-4o", messages=messages
    )

reply = chat.choices[0].message.content
print(f"Planner: \n{reply}")
messages.append({"role": "assistant", "content": reply})
action_list = []
action_list = reply.split(",")

class StringServiceServer(Node):

    def __init__(self):
        super().__init__('task_planner_service')
        self.srv = self.create_service(PlanningAction, 'task_planner_service', self.task_planner_callback)

    def task_planner_callback(self, request, response):
        global action_list
        self.get_logger().info('Incoming request: %s' % request.action_name)
        # Perform your task here
        while (len(action_list) == 0):
            pass
        
        self.get_logger().info('gpt response: %s' % action_list)
        # response.action_list = action_list
        # response.action_list = ["open hand", "move to goal", "pick object", "move to place", "place object", "return home"]
        if (request.action_name == "replan"):
            #response.action_list = ["grasp", "move to place", "ungrasp"]
            response.action_list = ["grasp", "move to place"]

        else:
            response.action_list = ["move to pick", "grasp", "move to place", "ungrasp"]
            #response.action_list = ["move to pick", "grasp", "move to place"]

        
        
        self.get_logger().info('responding with: %s' % response.action_list)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = StringServiceServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()




