import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String

from piper import PiperVoice, SynthesisConfig
import sounddevice as sd
import numpy as np

from ament_index_python import get_package_share_directory


class TTS_Playsound(Node):

    def __init__(self):
        super().__init__('sarai_tts_playsound_node')

        # Subscription to verbalize what is said on the tts_input topic
        self.speak_subscriber = self.create_subscription(String, 'tts_input', self.topic_speak_callback, 10)

        # Publisher to display robot speaking state
        self.display_robot_state_publisher = self.create_publisher(String, 'display_robot_state', 10)

        # Publisher to enable speech recognition after speaking is finished
        self.recognizer_state_publisher = self.create_publisher(Bool, 'recognizer_is_active', 10)

        # Load Piper voice
        voice_path = get_package_share_directory('sarai_tts_playsound') + "/voice_model/en_US-sam-medium.onnx"
        self.voice = PiperVoice.load(voice_path)
        self.syn_config = SynthesisConfig(volume=1.0,       # default volume
                                     length_scale=1.0,      # normal pace
                                     noise_scale=0.667,     # default audio variation
                                     noise_w_scale=0.8,     # default speaking variation
                                     normalize_audio=False, # use raw audio from voice
                                     )

    def speak(self, message):
        '''TODO:documentation'''

        stream = None

        try:
            for chunk in self.voice.synthesize(message, syn_config=self.syn_config):
                if stream is None:
                    stream = sd.OutputStream(
                        samplerate=chunk.sample_rate,
                        channels=chunk.sample_channels,
                        dtype='int16'
                    )
                    stream.start()

                # Convert the raw bytes to a NumPy array for sounddevice
                audio_array = np.frombuffer(chunk.audio_int16_bytes, dtype=np.int16)
                stream.write(audio_array)

        finally:
            if stream:
                stream.stop()
                stream.close()

    def topic_speak_callback(self, msg):
        '''TODO:documentation'''
        # Deactivate speech recognizer
        recognizer_state_msg = Bool()
        recognizer_state_msg.data = False
        self.recognizer_state_publisher.publish(recognizer_state_msg)

        # Display robot speaking state
        robot_state_msg = String()
        robot_state_msg.data = "speaking"
        self.display_robot_state_publisher.publish(robot_state_msg)

        self.speak(msg.data)

        # Display robot listening state
        robot_state_msg = String()
        robot_state_msg.data = "listening"
        self.display_robot_state_publisher.publish(robot_state_msg)

        # Reactivate speech recognizer
        recognizer_state_msg = Bool()
        recognizer_state_msg.data = True
        self.recognizer_state_publisher.publish(recognizer_state_msg)


def main(args=None):
    rclpy.init(args=args)

    tts_playsound_node = TTS_Playsound()

    rclpy.spin(tts_playsound_node)

    tts_playsound_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
