import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import time
import pygame
from pygame.locals import *
import numpy as np
import random

from ament_index_python import get_package_share_directory


class Display(Node):

    # DEFINE
    WHITE = (255, 255, 255)

    def __init__(self):
        super().__init__('pixelbot_display_node')

        # Initialize Pygame
        pygame.init()

        # Create Pygame window
        self.window = pygame.display.set_mode((0, 0), pygame.FULLSCREEN, display=1)

        # Do not show computer mouse
        pygame.mouse.set_visible(False)

        # Defines
        self.FPS = 60.0
        self.ANIMATION_INTERVAL = 3
        self.WIDTH, self.HEIGHT = self.window.get_size()
        self.RIGHT_EYE_CENTER_X = 0.25*self.WIDTH
        self.LEFT_EYE_CENTER_X = 0.75*self.WIDTH
        self.EYES_CENTER_Y = 0.5*self.HEIGHT
        self.RIGHT_EYE_CENTER = (self.RIGHT_EYE_CENTER_X, self.EYES_CENTER_Y)
        self.LEFT_EYE_CENTER = (self.LEFT_EYE_CENTER_X, self.EYES_CENTER_Y)
        self.EYE_RADIUS = 0.75*self.HEIGHT
        self.PROCESSING_STATE_INTERVAL = 2

        self.BLINKING_ANIMATION_EYES_HEIGHT = np.concatenate((np.linspace(self.EYE_RADIUS, 0, num=10),
                                                              np.linspace(0, self.EYE_RADIUS, num=10)))

        # Loading robot state images
        image_path = get_package_share_directory('pixelbot_display') + "/imgs/"
        drawing_image = pygame.image.load(image_path + "drawing_icon.png").convert_alpha()
        listening_image = pygame.image.load(image_path + "listening_icon.png").convert_alpha()
        processing_1_image = pygame.image.load(image_path + "robot_processing_icon1.png").convert_alpha()
        processing_2_image = pygame.image.load(image_path + "robot_processing_icon2.png").convert_alpha()
        processing_3_image = pygame.image.load(image_path + "robot_processing_icon3.png").convert_alpha()
        speaking_image = pygame.image.load(image_path + "robot_speaking_icon.png").convert_alpha()

        # Scale robot state images
        self.drawing_image = pygame.transform.scale(drawing_image, (self.WIDTH/10, self.WIDTH/10))
        self.listening_image = pygame.transform.scale(listening_image, (self.WIDTH/10, self.WIDTH/10))
        self.processing_1_image = pygame.transform.scale(processing_1_image, (self.WIDTH/10, self.WIDTH/10))
        self.processing_2_image = pygame.transform.scale(processing_2_image, (self.WIDTH/10, self.WIDTH/10))
        self.processing_3_image = pygame.transform.scale(processing_3_image, (self.WIDTH/10, self.WIDTH/10))
        self.processing_images_list = [self.processing_1_image, self.processing_2_image, self.processing_3_image]
        self.speaking_image = pygame.transform.scale(speaking_image, (self.WIDTH/10, self.WIDTH/10))

        # General purpose variables
        self.should_display = True
        self.animation_running = False
        self.counter_animation_frame = 0
        self.start_time_animation = time.time()
        self.robot_state = None
        self.previous_robot_state = None
        self.need_robot_state_update = False
        self.start_time_robot_processing_state = None
        self.robot_processing_animation_count = 0

        # Timer callback for the display
        timer_period = 1.0 / self.FPS # 60 fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Subscriber to display the state of the robot (listening, processing, speaking)
        self.display_robot_state_subscriber = self.create_subscription(String, 'display_robot_state', self.display_robot_state_callback, 10)

        # Initial image on window
        self.window.fill("white")
    
        self.right_eye = pygame.Rect(0, 0, 0, 0)
        self.left_eye = pygame.Rect(0, 0, 0, 0)
        
        self.right_eye.size = (self.EYE_RADIUS, self.EYE_RADIUS)
        self.left_eye.size = (self.EYE_RADIUS, self.EYE_RADIUS)
        
        self.right_eye.center = self.RIGHT_EYE_CENTER
        self.left_eye.center = self.LEFT_EYE_CENTER
        
        pygame.draw.ellipse(self.window, "black", self.right_eye)
        pygame.draw.ellipse(self.window, "black", self.left_eye)

        pygame.display.flip()

    def time_until_next_blink_refractory(self, mean_interval_s=4.0, t_min=0.25):
        """
        Returns the time (in seconds) until the next blink.
        Ensures a minimum spacing t_min (physiological refractory period).
        The remaining time is exponential with adjusted mean.
        """

        # Adjust rate so that mean total interval = mean_interval_s
        lam = 1.0 / max((mean_interval_s - t_min), 0.001)

        return t_min + random.expovariate(lam)

    def blinking(self):
        """
        Blinking animation.

        :param window: The Pygame window to draw on.
        :param counter_animation_frame: Counter keeping track of the animation frame.
        :param right_eye: Pygame Rect object representing the right eye.
        :param left_eye: Pygame Rect object representing the left eye.
        """

        self.right_eye.size = self.EYE_RADIUS, self.BLINKING_ANIMATION_EYES_HEIGHT[self.counter_animation_frame]
        self.left_eye.size = self.EYE_RADIUS, self.BLINKING_ANIMATION_EYES_HEIGHT[self.counter_animation_frame]

        self.right_eye.center = self.RIGHT_EYE_CENTER
        self.left_eye.center = self.LEFT_EYE_CENTER

        pygame.draw.ellipse(self.window, "black", self.right_eye)
        pygame.draw.ellipse(self.window, "black", self.left_eye)


    def timer_callback(self):
        """
        Main loop of the display.
        """

        # Event catching
        for event in pygame.event.get():
            if event.type == KEYDOWN and event.key == K_ESCAPE:
                self.should_display = False
    
        # Delay between each emotion animation:
        if self.animation_running == False and (time.time() - self.start_time_animation > self.ANIMATION_INTERVAL):
            self.ANIMATION_INTERVAL = self.time_until_next_blink_refractory()
            self.animation_running = True
        
        # Animation drawing
        if self.animation_running:
            # Fill the window with a white background
            self.window.fill("white")
    
            # Draw the eyes with the desired emotion
            self.blinking()
    
            # Update the frame counter and check for end of animation
            self.counter_animation_frame += 1
            if self.counter_animation_frame == len(self.BLINKING_ANIMATION_EYES_HEIGHT):
                self.counter_animation_frame = 0
                self.animation_running = False

                self.start_time_animation = time.time()

            self.need_robot_state_update = True

        # Robot state drawing
        if self.start_time_robot_processing_state:
            if time.time() - self.start_time_robot_processing_state > self.PROCESSING_STATE_INTERVAL:
                self.need_robot_state_update = True
   
        if self.need_robot_state_update:
            # Remove the previous state
            pygame.draw.rect(self.window, self.WHITE, (0, 0, self.WIDTH/10, self.WIDTH/10))
            pygame.draw.rect(self.window, self.WHITE, (self.WIDTH - self.WIDTH/10, 0, self.WIDTH/10, self.WIDTH/10))

            # Print robot state (if any)
            if self.robot_state == "listening":
                self.window.blit(self.listening_image, (self.WIDTH - self.listening_image.get_width(), 0))
                self.window.blit(self.drawing_image, (0, 0))

            if self.robot_state == "processing":
                if self.start_time_robot_processing_state is None:
                    self.start_time_robot_processing_state = time.time()

                if time.time() - self.start_time_robot_processing_state > self.PROCESSING_STATE_INTERVAL:
                    self.start_time_robot_processing_state = time.time()
                    self.robot_processing_animation_count +=1
                    if self.robot_processing_animation_count == 3:
                        self.robot_processing_animation_count = 0

                # check time when other emotion
                self.window.blit(self.processing_images_list[self.robot_processing_animation_count], (self.WIDTH - self.processing_3_image.get_width(), 0))
            
            if self.robot_state == "speaking":
                self.window.blit(self.speaking_image, (self.WIDTH - self.speaking_image.get_width(), 0))

        if self.animation_running or self.need_robot_state_update:    
            # Update the full display Surface to the screen
            self.need_robot_state_update = False
            pygame.display.flip()
    
    def display_robot_state_callback(self, msg):
        """TODO: documentation"""
        self.previous_robot_state = self.robot_state
        if msg.data == "None":
            self.robot_state = None
        else:
            self.robot_state = msg.data
            if self.previous_robot_state == "processing" and self.robot_state != "processing":
                self.start_time_robot_processing_state = None
                self.robot_processing_animation_count = 0

        if self.robot_state != self.previous_robot_state:
            self.need_robot_state_update = True

    def stop_pygame(self):
        """
        Stop Pygame.
        """

        pygame.quit()


def main(args=None):
    rclpy.init(args=args)

    display_node = Display()

    while rclpy.ok() and display_node.should_display:
        rclpy.spin_once(display_node)

    display_node.stop_pygame()

    display_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
