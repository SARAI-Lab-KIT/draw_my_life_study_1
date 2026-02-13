import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String, Empty

import cv2
from cv_bridge import CvBridge
import base64

import os
os.environ["OLLAMA_HOST"] = os.getenv("OLLAMA_HOST")

import ollama
from openai import OpenAI

from ament_index_python import get_package_share_directory


class VLM(Node):

    # Defines
    DRAWING_DESCRIPTION, DRAWING_QUESTION = 0, 1
    
    def __init__(self):
        super().__init__('pixelbot_vlm_node')

        # Subscriber to the recognized text topic
        self.recognized_text_subscriber = self.create_subscription(String, 'recognized_text', self.recognized_text_callback, 10)
        
        # Subscriber to the image topic from the drawing application
        self.drawing_subscriber = self.create_subscription(Image, 'drawings', self.drawing_callback, 10)

        # Subscriber to the child session folder path
        self.child_session_folder_path_subscriber = self.create_subscription(String, 'child_session_folder_path', self.child_session_folder_path_callback, 10)

        # Subscriber to reset the conversation history if a new draw my life session is started
        self.reset_subscriber = self.create_subscription(Empty, 'reset_drawing_session', self.reset_vlm_conversation_history_callback, 10)

        # Publisher to display robot processing state
        self.display_robot_state_publisher = self.create_publisher(String, 'display_robot_state', 10)
        
        # Publisher of the vlm output, to be said by the sarai tts node
        self.vlm_output_publisher = self.create_publisher(String, 'tts_input', 10)

        # Initialize the conversation history with the vlm persona
        vlm_persona_file_path = get_package_share_directory('pixelbot_vlm') + "/vlm_persona/vlm_persona_2.txt"
        with open(vlm_persona_file_path, 'r') as file:
            self.vlm_persona = file.read().replace('\n', ' ')

        self.conversation_history = [{"role": "system", "content": self.vlm_persona}]

        # Initialize the vlm model
        self.declare_parameter('vlm_model', 'kit.qwen3-vl-235b-a22b-instruct')
        self.VLM_MODEL = self.get_parameter('vlm_model').get_parameter_value().string_value

        # Initialize the vlm client
        if "sarai" not in self.VLM_MODEL:
            vlm_client_api_key = os.getenv("KI_TOOLBOX_API_KEY")
            self.openai_client = OpenAI(api_key=vlm_client_api_key, 
                                        base_url="https://ki-toolbox.scc.kit.edu/api/v1")

        # Initialize the cv bridge object
        self.cv_bridge = CvBridge()

        # General purpose variables
        self.child_session_folder_path = None
        self.most_recent_drawing = None
        self.drawing_number = 0


    def child_session_folder_path_callback(self, msg):
        self.child_session_folder_path = msg.data

    def reset_vlm_conversation_history_callback(self, msg):
        self.conversation_history = [{"role": "system", "content": self.vlm_persona}]
        self.most_recent_drawing = None
        self.drawing_number = 0

    def recognized_text_callback(self, msg):
        '''TODO: documentation'''
        if self.most_recent_drawing:
            self.vlm_processing(msg)
        else:
            # Display robot processing state
            robot_state_msg = String()
            robot_state_msg.data = "None"
            self.display_robot_state_publisher.publish(robot_state_msg)

    def drawing_callback(self, msg):
        '''TODO: documentation'''
        self.most_recent_drawing = msg
        #self.vlm_processing(msg)

    def convert_img_msg_for_vlm(self, img_msg):
        '''TODO: documentation'''
        cv_drawing_image = self.cv_bridge.imgmsg_to_cv2(img_msg)

        # Save drawing sent to vlm
        cv2.imwrite(self.child_session_folder_path + "/drawings/drawing_" + str(self.drawing_number) + ".png", cv_drawing_image)
        self.drawing_number += 1

        # Convert the image into a readable format for the vlm
        _, buffer = cv2.imencode(".jpg", cv_drawing_image)
        drawing_image_base64 = base64.b64encode(buffer).decode("utf-8")

        return drawing_image_base64

    def vlm_processing(self, msg):
        '''TODO: documentation'''
        # Display robot processing state
        robot_state_msg = String()
        robot_state_msg.data = "processing"
        self.display_robot_state_publisher.publish(robot_state_msg)

        drawing_image_base64 = self.convert_img_msg_for_vlm(self.most_recent_drawing)

        # Create new message to VLM
        # No speech received since 30 sec, send the latest version of the drawing
        if msg.data == "":
            self.get_logger().info("Analyzing image only")

            # Add the drawing to the conversation history
            # openai api
            if "sarai" not in self.VLM_MODEL:
                self.conversation_history.append({"role": "user",
                                                "content": [{"type": "image_url",
                                                            "image_url": {"url": f"data:image/jpeg;base64,{drawing_image_base64}"}}]})
            # ollama api
            else:
                self.conversation_history.append({"role": "user",
                                                  "content": "",
                                                  "images": [drawing_image_base64]})
            
            # Mention that image was sent to vlm in the transcript file
            with open(self.child_session_folder_path + "/transcript.txt", "a") as file:
                file.write("User: Drawing sent\n")

        # Recognized speech received, send it with latest version of the drawing 
        else:
            self.get_logger().info("Analyzing image and speech")
            
            # Add the recognized text and the drawing to the conversation history
            # openai api
            if "sarai" not in self.VLM_MODEL:
                self.conversation_history.append({"role": "user", 
                                                "content": [{"type": "text",
                                                            "text": msg.data},
                                                            {"type": "image_url",
                                                            "image_url":{"url": f"data:image/jpeg;base64,{drawing_image_base64}"}}]})
            # ollama api
            else:
                self.conversation_history.append({"role": "user",
                                                  "content": msg.data,
                                                  "images": [drawing_image_base64]})

            # Store recognized user speech in the transcript file
            with open(self.child_session_folder_path + "/transcript.txt", "a") as file:
                file.write("User: " + msg.data + "\n")

        # Give the image/recognized text as input of the vlm and get its output
        self.get_logger().info("send to vlm")
        if "sarai" not in self.VLM_MODEL:
            vlm_output = self.openai_client.chat.completions.create(model=self.VLM_MODEL, 
                                                                    messages=self.conversation_history)
            vlm_reply = vlm_output.choices[0].message.content
        else:
            vlm_output = ollama.chat(model="qwen3-vl:235b-a22b-instruct", 
                                     messages=self.conversation_history)
            vlm_reply = vlm_output["message"]["content"]


        # Update the conversation history with the vlm output
        self.conversation_history.append({"role": "assistant", "content": vlm_reply})

        # Print the vlm_reply
        self.get_logger().info(vlm_reply)

        # Save the vlm reply
        with open(self.child_session_folder_path + "/transcript.txt", "a") as file:
            file.write("VLM: " + vlm_reply + "\n")

        # Publish the vlm reply so that it is said by the tts node
        tts_msg = String()
        tts_msg.data = vlm_reply
        self.vlm_output_publisher.publish(tts_msg)


def main(args=None):
    rclpy.init(args=args)

    vlm_node = VLM()
    rclpy.spin(vlm_node)

    vlm_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
