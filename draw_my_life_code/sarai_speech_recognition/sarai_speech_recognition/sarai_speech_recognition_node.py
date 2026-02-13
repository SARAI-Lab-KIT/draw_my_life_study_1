import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String

import speech_recognition as sr
from speech_recognition.recognizers.whisper_local.faster_whisper import InitOptionalParameters

import time

# Removes all unnecessary error prints
import sounddevice


class Sarai_Speech_Recognition(Node):

    WRONG_SR_PATTERNS = ["", " ", ".", " ."]

    def __init__(self):
        super().__init__("sarai_speech_recognition_node")

        # Subscriber to control when to listen and when not
        self.recognizer_is_active = False
        self.recognizer_state_subscriber = self.create_subscription(Bool, 'recognizer_is_active', self.speech_recognizer_state_callback, 10)

        # Subscription to know what the robot last said
        self.speak_subscriber = self.create_subscription(String, 'tts_input', self.robot_last_utterance_callback, 10)
        self.robot_last_utterance = ""

        # Publisher to publish any recognized text
        self.recognized_text_publisher = self.create_publisher(String, 'recognized_text', 10)

        # Publisher to display robot listening state
        self.display_robot_state_publisher = self.create_publisher(String, 'display_robot_state', 10)

        # Timer to check inactivity of speech
        self.time_recognizer_activation = None
        timer_period = 2
        self.timer = self.create_timer(timer_period, self.check_speech_inactivity_callback)

        # Initialize the microphone and recognizer objects
        self.microphone = sr.Microphone() # TODO: device_index=MICROPHONE_INDEX, see SR Lib about RPi
        self.speech_recognizer = sr.Recognizer()

        # Calibrate the microphone for ambient noise levels
        self.get_logger().info("Calibrating the microphone to ambient noises. Please do not speak during the next 3 seconds.")
        with self.microphone as source:
            self.speech_recognizer.adjust_for_ambient_noise(source, duration=3)
        self.get_logger().info("Calibration finished.")

        # Increase the VAD threshold by a constant value, otherwise triggers too easily
        self.speech_recognizer.energy_threshold += 1000

        # Need to disable dynamic energy threshold
        # We don't want to change the already done calibration
        self.speech_recognizer.dynamic_energy_threshold = False

        # Seconds of non speaking audio before the speaking audio is considered a phrase. Default is 0.8
        self.speech_recognizer.pause_threshold = 1.0

        # Configuration for faster whisper (local, open-source)
        self.init_params = InitOptionalParameters(device="cpu", compute_type="int8")


    def recognition_callback(self, recognizer, audio):
        '''TODO: documentation'''
        try:
            # End of speech detected, start recognizing and then send to vlm
            robot_state_msg = String()
            robot_state_msg.data = "processing"
            self.display_robot_state_publisher.publish(robot_state_msg)

            recognized_speech = recognizer.recognize_faster_whisper(audio, language="en", model="base", init_options=self.init_params)
            
            # We check for wrong recognition patterns,
            # and recognition corresponding to the robot's previous output
            if recognized_speech not in self.WRONG_SR_PATTERNS and recognized_speech not in self.robot_last_utterance:
                # After a recognition, we deactivate the speech recognition
                self.stop_listening(wait_for_stop=False)
                self.recognizer_is_active = False
                self.time_recognizer_activation = None
                self.get_logger().info("Recognizer is now inactive (speech has been recognized).")
                
                # After a recognition, we send the recognized text to the vlm
                msg = String()
                msg.data = recognized_speech
                self.recognized_text_publisher.publish(msg)
            else:
                self.get_logger().info("Recognized speech in robot last utterance: " + recognized_speech)

        except sr.UnknownValueError:
            self.get_logger().info("Faster Whisper could not understand audio. Please try again!")
        except sr.RequestError as error:
            self.get_logger().info("Could not request results from Faster Whisper Speech Recognition service; {0}. Please try again!".format(error))

    def speech_recognizer_state_callback(self, msg):
        '''TODO: documentation'''
        
        # Recognizer was not active and there is a request to activate it
        if not self.recognizer_is_active and msg.data:
            self.stop_listening = self.speech_recognizer.listen_in_background(self.microphone, self.recognition_callback)
            self.get_logger().info("Recognizer is now active (from request).")
            robot_state_msg = String()
            robot_state_msg.data = "listening"
            self.display_robot_state_publisher.publish(robot_state_msg)
            self.time_recognizer_activation = time.time()

        # Recognizer is active and request to deactivate it
        elif self.recognizer_is_active and not msg.data:
            self.stop_listening(wait_for_stop=True)
            self.time_recognizer_activation = None
            self.get_logger().info("Recognizer is now inactive (from request).")

        # Update the new state of the recognizer
        self.recognizer_is_active = msg.data

    def check_speech_inactivity_callback(self):
        '''TODO: documentation'''
        if self.recognizer_is_active and self.time_recognizer_activation and time.time() - self.time_recognizer_activation > 30:
            self.stop_listening(wait_for_stop=True)
            self.recognizer_is_active = False
            self.time_recognizer_activation = None
            self.get_logger().info("Recognizer is now inactive (30s without finished speech).")

            # Inactivity detected, ask the vlm to process only the drawing
            msg = String()
            msg.data = ""
            self.recognized_text_publisher.publish(msg)
        
    def robot_last_utterance_callback(self, msg):
        """Keep track of the robot last utterance.
        This allows to check if the speech recognizer is recognizing
        the robot speech instead of the user speech.
        """

        self.robot_last_utterance = msg.data

    def destroy_node(self):
        """Ensure background listener is stopped cleanly."""
        if self.stop_listening is not None:
            self.get_logger().info("Stopping background listener.")
            self.stop_listening(wait_for_stop=True)

        super().destroy_node()
        

def main(args=None):
    rclpy.init(args=args)

    speech_recognition_node = Sarai_Speech_Recognition()

    rclpy.spin(speech_recognition_node)

    speech_recognition_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
