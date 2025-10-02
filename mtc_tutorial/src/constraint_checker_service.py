#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from task_planner_interfaces.srv import ConstraintCheck

import cv2
import base64
import os
from openai import OpenAI
import threading

SYSTEM_CONTEXT = """
You are a robot manipulator constraint checker who checks if certain constraints (provided as questions by the user) are being met and responds only in "yes" or "no".

For example, you might be asked:
- "Is the robot gripper holding the red object?"
- "Is the red object placed correctly on position A?"

You respond with "yes" or "no" only. This is absolutely integral to your function. Do not provide any additional information or explanations. Do not add full stop at the end of the response. Do not uppercase any character in the response. Do not add any disclaimers or qualifications. Just "yes" or "no".
"""
# Shared frame variable



class VLMServer(Node):

    def __init__(self):
        super().__init__('constraint_checker_service')
        self.get_logger().info("🚀 Starting VLM constraint checker service...")

        self.latest_frame = None
        self.url = "http://192.168.178.160:4747/video"
        self.thread = None
        self.cap = cv2.VideoCapture(self.url)
        self.client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

        if not self.cap.isOpened():
            self.get_logger().error("❌ Cannot connect to phone camera")
            exit()

        self.get_logger().info("✅ Camera connected. Ask a question (type 'quit' to exit).")

        # Run camera in background thread
        print("Starting camera thread")
        self.thread = threading.Thread(target=self.show_camera, daemon=True)
        self.thread.start()
        self.srv = self.create_service(ConstraintCheck, 'constraint_checker_service', self.constraint_checker_callback)

    def constraint_checker_callback(self, request, response):



        # Console Q&A loop
        self.get_logger().info('Incoming request: %s' % request.action_name)
        question = request.action_name

        if question.lower() == "quit":
            return response
        
        if self.latest_frame is None:
            self.get_logger().warn("⚠️ No frame available yet, try again.")
            return response

        # Encode the latest frame
        _, buffer = cv2.imencode(".png", self.latest_frame)  # Fixed the typo here
        img_bytes = buffer.tobytes()
        img_b64 = base64.b64encode(img_bytes).decode("utf-8")
        
        # Send question + image to GPT with system context
        vlm_response = self.client.chat.completions.create(
            model="gpt-4o",  # Vision-enabled
            messages=[
                {
                    "role": "system",
                    "content": SYSTEM_CONTEXT  # Add context here
                },
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": question},
                        {"type": "image_url", "image_url": {"url": f"data:image/png;base64,{img_b64}"}},
                    ],
                }
            ],
            max_tokens=200,
        )

        answer = vlm_response.choices[0].message.content
        if answer == "yes":
            response.constraint_met = True
        else:
            response.constraint_met = False
        self.get_logger().info('🤖 GPT response: %s' % answer)

        # cap.release()
        # cv2.destroyAllWindows()
        return response

    def show_camera(self):
        while True:
            ret, frame = self.cap.read()
            # print("Reading frame")
            if not ret:
                self.get_logger().error("❌ Failed to grab frame")
                break
            self.latest_frame = frame  # update latest frame
            cv2.imshow("Phone Camera Stream", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

def main(args=None):
    rclpy.init(args=args)
    node = VLMServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()




