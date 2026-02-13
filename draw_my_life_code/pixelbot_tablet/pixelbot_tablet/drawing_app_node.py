import sys
import pygame
import os
import time
from datetime import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from evdev import InputDevice, ecodes, list_devices
import threading
import json
from ament_index_python import get_package_share_directory


# main UI colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (150, 150, 150)
BLUE = (0, 120, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)

# Other define
BLUE_BORDER_THICKNESS = 5
DRAW, ERASE = 0, 1

def count_ink_pixels(surface):
    """
    Counts non-white pixels on the drawing surface.
    """

    surface_np = pygame.surfarray.pixels3d(surface)
    white = np.array(WHITE, dtype=surface_np.dtype)

    # A pixel is ink if at least one channel differs from white
    ink_mask = np.any(surface_np != white, axis=2)

    return int(np.count_nonzero(ink_mask))


class WacomPressureReader:
    def __init__(self, device_path):
        self.device = InputDevice(device_path)
        self.pressure = 0
        self.running = True
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

    def read_loop(self):
        for event in self.device.read_loop():
            if not self.running:
                break
            if event.type == ecodes.EV_ABS and event.code == ecodes.ABS_PRESSURE:
                self.pressure = event.value

    def stop(self):
        self.running = False


# class: Button
class IconButton:
    def __init__(self, undo_manager, camera, image_surface, position, palette=None, image_surface_disabled=None, type=None):
        self.undo_manager = undo_manager
        self.camera = camera
        self.image = image_surface
        self.image_gray = image_surface_disabled
        self.position = position
        self.rect = self.image.get_frect(topleft=position)
        self.palette = palette
        self.type = type

    # display icons of button in menu bar with fitting color and fitting border color
    def display(self, surface, border_color):
        if self.image_gray is not None:
            if self.undo_manager.current_index_surface_stack == 1 and self.type == "undo":
                surface.blit(self.image_gray, self.position)
            elif self.undo_manager.current_index_surface_stack == len(self.undo_manager.surface_stack) and self.type == "redo":
                surface.blit(self.image_gray, self.position)
            elif self.camera.zoom_scale == self.camera.min_zoom and self.type == "zoom out":
                surface.blit(self.image_gray, self.position)
            elif self.camera.zoom_scale == self.camera.max_zoom and self.type == "zoom in":
                surface.blit(self.image_gray, self.position)
            else:
                surface.blit(self.image, self.position)
        else:
            surface.blit(self.image, self.position)

        pygame.draw.rect(surface, border_color, self.rect, width=2, border_radius=int(min(self.rect.height, self.rect.width)/16))

    # check if user is hovering over icon button
    def check_click(self, mouse_pos):
        return self.rect.collidepoint(mouse_pos)

    # show option palette when icon button has been clicked
    def handle_click(self):
        if self.palette:
            self.palette.change_visibility()


# class: Palette
class Palette:
    def __init__(self, main_screen, options, type, pencil, window_width, pencil_size_list=None):
        self.main_screen = main_screen
        self.options = options
        self.type = type
        self.pencil = pencil
        self.visible = False
        self.pencil_size_list = pencil_size_list
        self.WINDOW_WIDTH = window_width

    # change visibility of palette instance
    def change_visibility(self):
        self.visible = not self.visible

    # hide palette
    def hide(self):
        self.visible = False

    # display option palette(color or pencil size)
    def display(self, surface, border_color):
        if self.visible:
            if self.type == "color":
                for option, position in self.options:
                    rect = pygame.Rect(*position, 1.5/100*self.WINDOW_WIDTH, 1.5/100*self.WINDOW_WIDTH)
                    pygame.draw.rect(surface, option, rect, border_radius=int(min(rect.height, rect.width)/16))
                    pygame.draw.rect(surface, border_color, rect, width=2, border_radius=int(min(rect.height, rect.width)/16))
            elif self.type == "size":
                for index in range(len(self.options)):
                    # Display the pencil image
                    self.main_screen.blit(
                        self.pencil_size_list[index], self.options[index][1])
                    # Draw the rectanle around the pencil image
                    rect = pygame.Rect(*self.options[index][1], 2.5/100*self.WINDOW_WIDTH, 2.5/100*self.WINDOW_WIDTH)
                    pygame.draw.rect(
                        surface, border_color, rect, width=2, border_radius=int(min(rect.height, rect.width)/16))

    # check what user chooses in palette and set choosen option
    def handle_palette_click(self, mouse_position):
        if self.visible:
            for option, position in self.options:
                if self.type == "color":
                    option_rectangle = pygame.Rect(position, (1.5/100*self.WINDOW_WIDTH, 1.5/100*self.WINDOW_WIDTH))
                    if option_rectangle.collidepoint(mouse_position):
                        self.pencil.color = option
                        self.hide()
                        break
                elif self.type == "size":
                    option_rectangle = pygame.Rect(position, (2.5/100*self.WINDOW_WIDTH, 2.5/100*self.WINDOW_WIDTH))
                    if option_rectangle.collidepoint(mouse_position):
                        self.pencil.tool_size = option
                        self.hide()
                        break


# class: Tool
class Tool:
    def __init__(self, camera, main_screen, color, tool_size, menu_height):
        self.camera = camera
        self.main_screen = main_screen
        self.color = color
        self.tool_size = tool_size
        self.menu_height = menu_height
        self.active = False
        self.last_position = None
        self.last_add_point_time = None
        self.time_threshold = 3.5
        self.button_down_time = None
        self.ink_pixel_before_erased = None

    # method to draw a rather smooth line
    def draw_smooth_line(self, surface, start_position, end_position):
        if start_position == end_position:
            pygame.draw.circle(surface, self.color,
                               start_position, self.tool_size // 2)
        else:
            pygame.draw.line(surface, self.color, start_position,
                             end_position, self.tool_size)
            pygame.draw.circle(surface, self.color,
                               start_position, self.tool_size // 2.5)
            pygame.draw.circle(surface, self.color,
                               end_position, self.tool_size // 2.5)

    # method to handle drawing, erasing, moving the drawing and capturing stoke coordinates
    def handle_event(self, event, surface, pencil, eraser, move_tool, active_tool, menu_height, current_stroke, activity_start_time, current_pressure=None):
        number_erased_pixels = 0
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            self.active = True
            self.last_position = event.pos
            self.button_down_time = time.time()

            if active_tool == eraser:
                self.ink_pixel_before_erased = count_ink_pixels(surface)

        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.active = False
            self.last_position = None

            if active_tool == eraser:
                ink_pixel_after_erased = count_ink_pixels(surface)
                number_erased_pixels = self.ink_pixel_before_erased - ink_pixel_after_erased

        elif event.type == pygame.MOUSEMOTION and self.active:
            mouse_position = pygame.math.Vector2(event.pos)

            if active_tool == move_tool:
                current_mouse_position = mouse_position
                difference = (self.last_position - current_mouse_position) / self.camera.zoom_scale
                self.camera.offset += difference
                self.last_position = current_mouse_position
                self.camera.limit_offset()

            elif active_tool in (pencil, eraser):
                adjusted_position = (mouse_position / self.camera.zoom_scale + self.camera.offset -
                                     pygame.math.Vector2(0, self.menu_height) / self.camera.zoom_scale)

                if self.last_position and event.pos[1] >= menu_height:
                    adjusted_last = (pygame.math.Vector2(self.last_position) / self.camera.zoom_scale + self.camera.offset - 
                                     pygame.math.Vector2(0, self.menu_height) / self.camera.zoom_scale)
                    self.draw_smooth_line(surface, adjusted_last, adjusted_position)
                    self.camera.drawing_version += 1

                    # Recording of stroke color, size and points
                    if active_tool == pencil:
                        self.last_add_point_time = time.time()
                        if current_stroke == None:
                            current_stroke = {"pen_color": self.color,
                                              "pen_size": self.tool_size,
                                              "points": [(int(adjusted_last.x), int(adjusted_last.y)), (int(adjusted_position.x), int(adjusted_position.y))],
                                              "timestamps": [self.button_down_time - activity_start_time, self.last_add_point_time - activity_start_time]}
                            if current_pressure is not None:
                                current_stroke["pressure_values"] = [current_pressure, current_pressure]
                        else:
                            current_stroke["points"].append((int(adjusted_position.x), int(adjusted_position.y)))
                            current_stroke["timestamps"].append(self.last_add_point_time - activity_start_time)
                            if current_pressure is not None:
                                current_stroke["pressure_values"].append(current_pressure)

                self.last_position = mouse_position
                    
        return current_stroke, number_erased_pixels
    
    # check if user hasnt drawn in the last time threshold
    def check_inactivity(self):
        if self.last_add_point_time and time.time() - self.last_add_point_time > self.time_threshold:
            self.last_add_point_time = None
            return True
        else:
            return False

    # display fitting eraser border, depending on zoom
    def display_eraser_border(self, active_tool, eraser, menu_height):
        if active_tool == eraser:
            mouse_position = pygame.mouse.get_pos()
            if mouse_position[1] >= menu_height + 5:
                zoom = self.camera.zoom_scale
                radius = int(self.tool_size * zoom / 2.5)
                pygame.draw.circle(
                    self.main_screen, BLACK, mouse_position, radius + 2, width=2)
                pygame.draw.circle(
                    self.main_screen, WHITE, mouse_position, radius, width=2)

    # display fitting tool icon border color, depends on wether tool is activated
    def display_borders_tool(self, rectangle, active_tool):
        if self == active_tool:
            pygame.draw.rect(self.main_screen, BLUE, rectangle, width=2, border_radius=int(min(rectangle.height, rectangle.width)/16))
        else:
            pygame.draw.rect(self.main_screen, GRAY, rectangle, width=2, border_radius=int(min(rectangle.height, rectangle.width)/16))


# class: Start screen
class StartScreenDocumentation:
    def __init__(self, screen, welcome_image_path, button_image_path, exit_image_path,
                 background_image_path, window_height, window_width, clock, undo_manager, camera, save_manager, greeting_message_publisher):
        self.window_height = window_height
        self.window_width = window_width
        self.clock = clock
        self.undo_manager = undo_manager
        self.camera = camera
        self.save_manager = save_manager
        self.greeting_message_publisher = greeting_message_publisher
        self.screen = screen
        self.startscreen_done = False
        self.documentation_done = False

        welcome_image_size = (self.window_width//2, self.window_height//3)
        self.welcome_image = pygame.transform.smoothscale(pygame.image.load(
            welcome_image_path).convert_alpha(), welcome_image_size)
        self.start_button_image = pygame.transform.smoothscale(pygame.image.load(
            button_image_path).convert_alpha(), (self.window_width//10, self.window_height//10))
        self.background_image = pygame.transform.smoothscale(
            pygame.image.load(background_image_path).convert_alpha(), (self.window_width, self.window_height))
        
        self.start_button_position = (self.window_width // 2 - self.start_button_image.get_width() // 2, self.window_height // 2 + self.start_button_image.get_height() // 2)
        self.start_button = IconButton(self.undo_manager, self.camera, self.start_button_image, (self.start_button_position))

        self.exit_button_position = (self.window_width - self.window_width // 30, 2/100*self.window_height)
        self.exit_button = pygame.transform.smoothscale(pygame.image.load(
            exit_image_path).convert_alpha(), (2/100*self.window_width, 2/100*self.window_width))
        
        self.documentation_image = pygame.transform.smoothscale(pygame.image.load(os.path.join(
            'src', 'pixelbot_tablet', "images", "Documentation Image.png")).convert(), (self.window_width, self.window_height))

    # display documentation and handle close button
    def show_documentation(self):
        self.documentation_done = False
        while not self.documentation_done:
            self.clock.tick(10)
            for event in pygame.event.get():
                if event.type == pygame.MOUSEBUTTONUP:
                    if self.exit_button.get_frect(topleft=(self.window_width - self.window_width // 20, 6/100*self.window_height)).collidepoint(event.pos):
                        self.documentation_done = True

            self.screen.blit(self.documentation_image, (0, 0))
            self.screen.blit(self.exit_button, (self.window_width - self.window_width // 20, 6/100*self.window_height))
            pygame.display.update()

    # display startscreen with hover effect of start button
    def display(self):
        self.screen.fill(WHITE)
        self.screen.blit(self.background_image)
        self.screen.blit(self.welcome_image,
                         (self.window_width // 2 - self.welcome_image.get_width() // 2,
                          self.window_height // 2 - self.welcome_image.get_height() // 2 - self.window_height // 8)
                         )
        hovered = self.start_button.check_click(pygame.mouse.get_pos())
        if hovered:
            self.start_button.display(self.screen, BLUE)
        elif not hovered:
            self.start_button.display(self.screen, BLACK)
        self.screen.blit(self.exit_button, self.exit_button_position)

    # handle clicks in startscreen
    def handle_start_event(self, event):
        if event.type == pygame.MOUSEBUTTONUP:
            if self.start_button.check_click(event.pos):
                self.startscreen_done = True
            elif self.exit_button.get_frect(topleft=self.exit_button_position).collidepoint(event.pos):
                popup_confirmation = PopupWindow(
                    self.screen, pygame.font.SysFont('arial', 25), self.clock)
                answer = popup_confirmation.ask_confirmation(
                    "Do you really want to exit?")
                if answer == "yes":
                    popup_close = PopupWindow(
                        self.screen, pygame.font.SysFont('arial', 25), self.clock)
                    password = popup_close.handle_input(
                        "Enter password:", is_password=True)
                    if password == "123":
                        # Add clean closing of all the nodes
                        return True
        return False

    # run start screen, show documentation once and ask for child name
    def run(self):
        self.startscreen_done = False
        popup = PopupWindow(
            self.screen, pygame.font.SysFont('arial', 25), self.clock)

        while not self.startscreen_done:
            self.clock.tick(10)
            for event in pygame.event.get():
                if self.handle_start_event(event):
                    return False
            self.display()
            pygame.display.update()
        self.child_name = popup.handle_input("What's your name?")
        self.save_manager.create_saving_folder(self.child_name)
        self.show_documentation()

        # Greet the child by sending a speak request to the tts node
        greeting_msg = String()
        greeting_msg.data = "Hello " + self.child_name + "! Today, we are going to do a draw my life activity. The goal of this activity is to tell a story about yourself. You can do so by drawing your story on the tablet, talking to me, or both! Let's start and have fun together!"
        self.greeting_message_publisher.publish(greeting_msg)

        return True


# class: Undo manager: manages undo and redo functionality
class UndoManager:
    def __init__(self):
        self.surface_stack = []
        self.current_index_surface_stack = 0

        self.stroke_data_stack = []
        self.current_index_stroke_data_stack = 0

        self.last_drawing_action = None

        self.undone_erased_stroke_list = []

    def reset(self):
        self.surface_stack.clear()
        self.stroke_data_stack.clear()
        self.undone_erased_stroke_list.clear()
        self.current_index_surface_stack = 0
        self.current_index_stroke_data_stack = 0
        self.last_drawing_action = None

    # add copy of current drawing interface to surface_stack.
    def append_drawing(self, surface):
        # delete all newer drawings than current drawing when user draws something new
        if self.current_index_surface_stack < len(self.surface_stack):
            self.surface_stack = self.surface_stack[:self.current_index_surface_stack]

        self.surface_stack.append(surface.copy())
        self.current_index_surface_stack += 1

    def append_stroke(self, stroke):
        # if user draws something new, you can't redo
        if self.current_index_stroke_data_stack < len(self.stroke_data_stack):
            # store the undone strokes
            self.undone_erased_stroke_list = self.undone_erased_stroke_list + self.stroke_data_stack[self.current_index_stroke_data_stack:]

            self.stroke_data_stack = self.stroke_data_stack[:self.current_index_stroke_data_stack]

        self.stroke_data_stack.append(stroke.copy())
        self.current_index_stroke_data_stack += 1

    # check if the given surface is the same as the newest saved drawing in the undomanager surface_stack
    def compare_drawing_surface_to_last(self, surface):
        # Size must match
        if surface.get_size() != self.surface_stack[self.current_index_surface_stack - 1].get_size():
            return False
        
        # Pixel content must match
        surface_bytes = pygame.image.tobytes(surface, "RGBA")
        previous_surface_bytes = pygame.image.tobytes(self.surface_stack[self.current_index_surface_stack - 1], "RGBA")
        if surface_bytes != previous_surface_bytes:
            return False
        
        return True

    def undo(self, surface):
        if self.current_index_surface_stack > 1:
            self.current_index_surface_stack -= 1
            surface.blit(self.surface_stack[self.current_index_surface_stack - 1], (0, 0))
        if self.current_index_stroke_data_stack > 0 and self.last_drawing_action == DRAW:
            self.current_index_stroke_data_stack -= 1

    def redo(self, surface):
        if self.current_index_surface_stack < len(self.surface_stack):
            surface.blit(self.surface_stack[self.current_index_surface_stack], (0, 0))
            self.current_index_surface_stack += 1
        if self.current_index_stroke_data_stack < len(self.stroke_data_stack) and self.last_drawing_action == DRAW:
            self.current_index_stroke_data_stack += 1


# class: Camera
class Camera():
    def __init__(self, window_width, window_height, drawing_surface_width, drawing_surface_height,
                 menu_height, drawing_surface):
        self.display_surface = pygame.display.get_surface()
        self.window_width = window_width
        self.window_height = window_height
        self.drawing_surface_width = drawing_surface_width
        self.drawing_surface_height = drawing_surface_height
        self.menu_height = menu_height
        self.drawing_surface = drawing_surface
        self.last_zoom_scale = None
        self.cached_scaled_surface = None
        self.drawing_version = 0
        self.cached_drawing_version = -1

        # camera offset
        self.offset = pygame.math.Vector2(0, 0)

        self.zoom_scale = 1
        self.min_zoom = 1
        self.max_zoom = 1.3

    def reset(self):
        self.last_zoom_scale = None
        self.cached_scaled_surface = None
        self.drawing_version = 0
        self.cached_drawing_version = -1

        self.offset = pygame.math.Vector2(0, 0)

        self.zoom_scale = 1


    # handle zoom functionality
    def zoom(self, zoom_amount):
        screen_center = pygame.math.Vector2(
            self.window_width // 2, (self.window_height - self.menu_height)//2)
        menu_offset = pygame.math.Vector2(0, self.menu_height)

        old_zoom = self.zoom_scale
        self.zoom_scale = max(self.min_zoom, min(
            self.zoom_scale + zoom_amount, self.max_zoom))

        old_center = (screen_center + self.offset *
                      old_zoom - menu_offset) / old_zoom
        self.offset = old_center - \
            (screen_center - menu_offset)/self.zoom_scale
        self.last_zoom_scale = None

    def zoom_in(self):
        self.zoom(0.1)

    def zoom_out(self):
        self.zoom(-0.1)

    # limit zoom
    def limit_offset(self):
        max_x = self.drawing_surface_width - \
            (self.window_width / self.zoom_scale)
        max_y = self.drawing_surface_height - \
            ((self.window_height - self.menu_height) / self.zoom_scale)

        self.offset.x = max(0, min(self.offset.x, max_x))
        self.offset.y = max(0, min(self.offset.y, max_y))

    def update_scaled_surface(self):
        if self.zoom_scale == 1:
            self.cached_scaled_surface = self.drawing_surface
        else:
            self.cached_scaled_surface = pygame.transform.smoothscale(
                self.drawing_surface, (int(self.drawing_surface_width * self.zoom_scale),
                                       int(self.drawing_surface_height * self.zoom_scale)))
        self.last_zoom_scale = self.zoom_scale
        self.cached_drawing_version = self.drawing_version

    # display scaled drawing surface with offset
    def custom_display(self):
        if (self.zoom_scale != self.last_zoom_scale or
            self.cached_scaled_surface is None or
                self.cached_drawing_version != self.drawing_version):
            self.update_scaled_surface()

        scaled_offset = self.offset * self.zoom_scale
        drawing_surface_offset = pygame.math.Vector2(0, self.menu_height) - scaled_offset

        self.display_surface.blit(
            self.cached_scaled_surface, drawing_surface_offset)


# Class: Save Manager: manage saving the drawing of user with their name and time of saving
# Also saves the data related to the drawing self-disclosure width
class SaveManager():
    def __init__(self, surface, child_session_folder_path_publisher):
        self.surface = surface
        self.drawing_surface_width = self.surface.get_width()
        self.drawing_surface_height = self.surface.get_height()
        self.child_session_folder_path_publisher = child_session_folder_path_publisher
        self.session_path = None
        self.drawing_path = None
        self.session_number = None

    def create_saving_folder(self, child_name):
        share_dir = get_package_share_directory('pixelbot_tablet')
        src_dir = share_dir.replace('/install/', '/src/').split('/share/')[0]
        child_folder_path = src_dir + "/saved_drawings/" + child_name
        os.makedirs(child_folder_path, exist_ok=True)

        self.session_number = len(os.listdir(child_folder_path))
        self.session_path = os.path.join(child_folder_path, f"session_{self.session_number}")
        os.makedirs(self.session_path, exist_ok=True)

        self.drawing_path = os.path.join(self.session_path, "drawings")
        os.makedirs(self.drawing_path, exist_ok=True)

        child_session_folder_path_msg = String()
        child_session_folder_path_msg.data = self.session_path
        self.child_session_folder_path_publisher.publish(child_session_folder_path_msg)

    # save drawing in corresponding child folder and add saving time
    def save_drawing(self, activity_start_time):
        copy_surface = self.surface.copy()
        
        date = datetime.now().strftime("%d-%m-%Y")
        timestamp = round(time.time() - activity_start_time, 1)
        drawing_save_path = os.path.join(
            self.drawing_path, f"session{self.session_number}_{date}_{timestamp}.png")
        pygame.image.save(copy_surface, drawing_save_path)

    def save_stroke_data(self, stroke_data_array, undone_erased_stroke_data_array, activity_time, net_erased_pixels):
        """TODO: Documentation"""

        data = {"activity_time": activity_time,
                "net_erased_pixels": net_erased_pixels,
                "strokes": stroke_data_array,
                "undone_erased_strokes": undone_erased_stroke_data_array}

        with open(self.session_path + "/strokes.json", "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)


class PopupWindow:
    def __init__(self, screen_surface, font, clock, title="Wait!"):
        self.screen_surface = screen_surface
        self.font = font
        self.clock = clock
        self.title = title
        self.width = self.screen_surface.get_width() // 6
        self.height = self.screen_surface.get_height() // 8
        self.input_text = ""
        self.active_input = False
        self.result = None

        # display only popup rectangle
        self.display_rectangle = pygame.Rect(
            (screen_surface.get_width() - self.width) // 2,
            screen_surface.get_height() // 2 + self.width // 6,
            self.width,
            self.height)
        
        # display yes and no button
        self.button_yes = pygame.Rect(self.display_rectangle.x + 10/100*self.display_rectangle.width, 
                                      self.display_rectangle.y + 60/100*self.display_rectangle.height, 
                                      30/100*self.display_rectangle.width, 
                                      27/100*self.display_rectangle.height)
        
        self.button_no = pygame.Rect(self.display_rectangle.x + 60/100*self.display_rectangle.width, 
                                     self.display_rectangle.y + 60/100*self.display_rectangle.height, 
                                     30/100*self.display_rectangle.width, 
                                     27/100*self.display_rectangle.height)

    # display some text
    def draw_text(self, text, position, color=BLACK):
        rendered_text = self.font.render(text, True, color)
        self.screen_surface.blit(rendered_text, position)

    # display popup window with text
    def display_rectangle_on(self, message, color=(220, 220, 220), border_color=BLACK):
        # Draw main rect
        pygame.draw.rect(self.screen_surface, color, self.display_rectangle)

        # Draw main rect border
        pygame.draw.rect(self.screen_surface, border_color, self.display_rectangle, 2)

        self.draw_text(self.title, (self.display_rectangle.x + 5/100*self.display_rectangle.width, 
                                    self.display_rectangle.y + 10/100*self.display_rectangle.height))
        
        self.draw_text(message, (self.display_rectangle.x + 5/100*self.display_rectangle.width, 
                                 self.display_rectangle.y + 32/100*self.display_rectangle.height))

    # ask for comfirmation when trying to close the application
    def ask_confirmation(self, message):
        while self.result is None:
            self.clock.tick(20)
            for event in pygame.event.get():

                if event.type == pygame.MOUSEBUTTONUP:
                    if self.button_yes.collidepoint(event.pos):
                        self.result = "yes"
                    elif self.button_no.collidepoint(event.pos):
                        self.result = "no"

            self.display_rectangle_on(message)
            pygame.draw.rect(self.screen_surface, GREEN, self.button_yes)
            pygame.draw.rect(self.screen_surface, RED, self.button_no)
            self.draw_text("yes", (self.button_yes.x + 32/100*self.button_yes.width, self.button_yes.y + 14/100*self.button_yes.height))
            self.draw_text("no", (self.button_no.x + 38/100*self.button_no.width, self.button_no.y + 14/100*self.button_no.height))
            pygame.display.update()

        return self.result

    # show window to get input from user
    def handle_input(self, message, is_password=False):
        input_box = pygame.Rect(self.display_rectangle.x + 5/100*self.display_rectangle.width, 
                                self.display_rectangle.y + 60/100*self.display_rectangle.height, 
                                90/100*self.display_rectangle.width, 
                                25/100*self.display_rectangle.height)
        
        self.input_text = ""
        self.active_input = True
        MAX_CHAR_LIMIT = 12

        while self.result is None:
            self.clock.tick(10)
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN and self.active_input:
                    if event.key == pygame.K_RETURN:
                        self.result = self.input_text
                    elif event.key == pygame.K_BACKSPACE:
                        self.input_text = self.input_text[:-1]
                    else:
                        if len(self.input_text) < MAX_CHAR_LIMIT:
                            self.input_text += event.unicode

            self.display_rectangle_on(message)
            pygame.draw.rect(self.screen_surface, WHITE, input_box)
            pygame.draw.rect(self.screen_surface, BLACK, input_box, 2)

            display_text = "*" * \
                len(self.input_text) if is_password else self.input_text
            self.draw_text(display_text, (input_box.x + 2/100*input_box.width, input_box.y + 25/100*input_box.height))

            pygame.display.update()

        return self.result.lower()


#############################################################################################################################################


class DrawingApplicationNode(Node):

    def __init__(self):
        super().__init__('drawing_application_node')
        pygame.init()

        self.cv_bridge = CvBridge()
        self.drawing_publisher = self.create_publisher(Image, 'drawings', 10)

        self.activity_started = False

        self.child_session_folder_path_publisher = self.create_publisher(String, 'child_session_folder_path', 10)

        self.recognizer_state_publisher = self.create_publisher(Bool, 'recognizer_is_active', 10)

        # Publisher to display robot processing state
        self.display_robot_state_publisher = self.create_publisher(String, 'display_robot_state', 10)

        # Publisher of the greeting message, to be said by the sarai tts node
        self.greeting_message_publisher = self.create_publisher(String, 'tts_input', 10)

        # Publisher to let the other nodes to reset variables when a new draw my life session is started
        self.reset_publisher = self.create_publisher(Empty, 'reset_drawing_session', 10)

        # Parameter for pen pressure recording
        self.declare_parameter('pressure_recording', 'enabled')
        self.pressure_recording = self.get_parameter('pressure_recording').get_parameter_value().string_value

        if self.pressure_recording == "enabled":
            device_path = None
            for path in list_devices():
                dev = InputDevice(path)
                if "Wacom" in dev.name and "Pen" in dev.name:
                    device_path = path
                    break
            self.pressure_reader = WacomPressureReader(device_path)

        self.activity_start_time = None

        # Variables for logging the amount of pixel removing done by the user
        self.net_erased_pixels = 0

        # Variables to store the drawing self-disclosure width analysis
        self.current_stroke = None

        self.main_screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN, display=0)
        pygame.display.set_caption('drawing interface')
        self.main_screen.fill(WHITE)

        # Defines
        self.WINDOW_WIDTH, self.WINDOW_HEIGHT = self.main_screen.get_size()
        self.GENERAL_ICON_SIZE = (2.5/100*self.WINDOW_WIDTH, 2.5/100*self.WINDOW_WIDTH)
        self.MENU_HEIGHT = 10/100*self.WINDOW_HEIGHT
        self.DRAWING_SURFACE_WIDTH, self.DRAWING_SURFACE_HEIGHT = self.WINDOW_WIDTH, self.WINDOW_HEIGHT - self.MENU_HEIGHT

        # different class instances needed for general functionalities
        self.drawing_surface = pygame.Surface((self.DRAWING_SURFACE_WIDTH, self.DRAWING_SURFACE_HEIGHT))

        self.undo_manager = UndoManager()
        self.save_manager = SaveManager(self.drawing_surface, self.child_session_folder_path_publisher)
        self.camera = Camera(self.WINDOW_WIDTH, self.WINDOW_HEIGHT, self.DRAWING_SURFACE_WIDTH, self.DRAWING_SURFACE_HEIGHT,
                             self.MENU_HEIGHT, self.drawing_surface)

        self.pencil = Tool(self.camera, self.main_screen, BLACK, 10, self.MENU_HEIGHT)
        self.eraser = Tool(self.camera, self.main_screen, WHITE,  40, self.MENU_HEIGHT)
        self.move_tool = Tool(self.camera, self.main_screen, WHITE, 2, self.MENU_HEIGHT)
        self.active_tool = self.pencil

        # icon positions
        self.pencil_icon_position = (1.5/100*self.WINDOW_WIDTH, self.MENU_HEIGHT/2 - self.GENERAL_ICON_SIZE[0]/2)
        self.eraser_icon_position = (4.5/100*self.WINDOW_WIDTH, self.MENU_HEIGHT/2 - self.GENERAL_ICON_SIZE[0]/2)
        self.move_tool_icon_position = (7.5/100*self.WINDOW_WIDTH, self.MENU_HEIGHT/2 - self.GENERAL_ICON_SIZE[0]/2)
        self.color_icon_position = (10.5/100*self.WINDOW_WIDTH, self.MENU_HEIGHT/2 - self.GENERAL_ICON_SIZE[0]/2)
        self.pencil_size_icon_position = (13.5/100*self.WINDOW_WIDTH, self.MENU_HEIGHT/2 - self.GENERAL_ICON_SIZE[0]/2)
        self.zoom_in_icon_position = (16.5/100*self.WINDOW_WIDTH, self.MENU_HEIGHT/2 - self.GENERAL_ICON_SIZE[0]/2)
        self.zoom_out_icon_position = (19.5/100*self.WINDOW_WIDTH, self.MENU_HEIGHT/2 - self.GENERAL_ICON_SIZE[0]/2)
        self.undo_icon_position = (22.5/100*self.WINDOW_WIDTH, self.MENU_HEIGHT/2 - self.GENERAL_ICON_SIZE[0]/2)
        self.redo_icon_position = (25.5/100*self.WINDOW_WIDTH, self.MENU_HEIGHT/2 - self.GENERAL_ICON_SIZE[0]/2)
        self.save_icon_position = (self.WINDOW_WIDTH - 3.5/100*self.WINDOW_WIDTH, self.MENU_HEIGHT/2 - self.GENERAL_ICON_SIZE[0]/2)
        self.documentation_icon_position = (self.WINDOW_WIDTH - 6.5/100*self.WINDOW_WIDTH, self.MENU_HEIGHT/2 - self.GENERAL_ICON_SIZE[0]/2)

        # RGB values for the color palette and the position of the colors
        self.color_palette = [
            # first row
            ((102, 0, 0), (32/100*self.WINDOW_WIDTH, 0.5/100*self.WINDOW_HEIGHT)),          # dark red
            ((102, 51, 0), (33.75/100*self.WINDOW_WIDTH, 0.5/100*self.WINDOW_HEIGHT)),      # dark orange / brown
            ((102, 102, 0), (35.5/100*self.WINDOW_WIDTH, 0.5/100*self.WINDOW_HEIGHT)),      # olive
            ((51, 102, 0), (37.25/100*self.WINDOW_WIDTH, 0.5/100*self.WINDOW_HEIGHT)),      # dark lime green
            ((0, 102, 0), (39/100*self.WINDOW_WIDTH, 0.5/100*self.WINDOW_HEIGHT)),          # dark green
            ((0, 102, 51), (40.75/100*self.WINDOW_WIDTH, 0.5/100*self.WINDOW_HEIGHT)),      # teal green
            ((0, 102, 102), (42.5/100*self.WINDOW_WIDTH, 0.5/100*self.WINDOW_HEIGHT)),      # teal
            ((0, 51, 102), (44.25/100*self.WINDOW_WIDTH, 0.5/100*self.WINDOW_HEIGHT)),      # dark sky blue
            ((0, 0, 102), (46/100*self.WINDOW_WIDTH, 0.5/100*self.WINDOW_HEIGHT)),          # dark blue
            ((51, 0, 102), (47.75/100*self.WINDOW_WIDTH, 0.5/100*self.WINDOW_HEIGHT)),      # dark violet
            ((102, 0, 102), (49.5/100*self.WINDOW_WIDTH, 0.5/100*self.WINDOW_HEIGHT)),      # dark magenta
            ((102, 0, 51), (51.25/100*self.WINDOW_WIDTH, 0.5/100*self.WINDOW_HEIGHT)),      # dark pinkish magenta
            ((32, 32, 32), (53/100*self.WINDOW_WIDTH, 0.5/100*self.WINDOW_HEIGHT)),         # very dark gray
            ((0, 0, 0), (54.75/100*self.WINDOW_WIDTH, 0.5/100*self.WINDOW_HEIGHT)),         # black

            # second row
            ((255, 0, 0), (32/100*self.WINDOW_WIDTH, 3.75/100*self.WINDOW_HEIGHT)),         # pure red
            ((255, 128, 0), (33.75/100*self.WINDOW_WIDTH, 3.75/100*self.WINDOW_HEIGHT)),    # orange
            ((255, 255, 0), (35.5/100*self.WINDOW_WIDTH, 3.75/100*self.WINDOW_HEIGHT)),     # yellow
            ((128, 255, 0), (37.25/100*self.WINDOW_WIDTH, 3.75/100*self.WINDOW_HEIGHT)),    # lime green
            ((0, 255, 0), (39/100*self.WINDOW_WIDTH, 3.75/100*self.WINDOW_HEIGHT)),         # green
            ((0, 255, 128), (40.75/100*self.WINDOW_WIDTH, 3.75/100*self.WINDOW_HEIGHT)),    # light greenish-cyan
            ((0, 255, 255), (42.5/100*self.WINDOW_WIDTH, 3.75/100*self.WINDOW_HEIGHT)),     # cyan (aqua)
            ((0, 128, 255), (44.25/100*self.WINDOW_WIDTH, 3.75/100*self.WINDOW_HEIGHT)),    # light blue
            ((0, 0, 255), (46/100*self.WINDOW_WIDTH, 3.75/100*self.WINDOW_HEIGHT)),         # blue
            ((127, 0, 255), (47.75/100*self.WINDOW_WIDTH, 3.75/100*self.WINDOW_HEIGHT)),    # violet
            ((255, 0, 255), (49.5/100*self.WINDOW_WIDTH, 3.75/100*self.WINDOW_HEIGHT)),     # magenta (fuchsia)
            ((255, 0, 127), (51.25/100*self.WINDOW_WIDTH, 3.75/100*self.WINDOW_HEIGHT)),    # pinkish magenta
            ((128, 128, 128), (53/100*self.WINDOW_WIDTH, 3.75/100*self.WINDOW_HEIGHT)),     # medium gray

            # third row
            ((255, 153, 153), (32/100*self.WINDOW_WIDTH, 7/100*self.WINDOW_HEIGHT)),        # light red / salmon
            ((255, 204, 153), (33.75/100*self.WINDOW_WIDTH, 7/100*self.WINDOW_HEIGHT)),     # peach
            ((255, 255, 153), (35.5/100*self.WINDOW_WIDTH, 7/100*self.WINDOW_HEIGHT)),      # light yellow
            ((204, 255, 153), (37.25/100*self.WINDOW_WIDTH, 7/100*self.WINDOW_HEIGHT)),     # light lime green
            ((153, 255, 153), (39/100*self.WINDOW_WIDTH, 7/100*self.WINDOW_HEIGHT)),        # light green
            ((153, 255, 204), (40.75/100*self.WINDOW_WIDTH, 7/100*self.WINDOW_HEIGHT)),     # light aquamarine
            ((153, 255, 255), (42.5/100*self.WINDOW_WIDTH, 7/100*self.WINDOW_HEIGHT)),      # light cyan
            ((153, 204, 255), (44.25/100*self.WINDOW_WIDTH, 7/100*self.WINDOW_HEIGHT)),     # light sky blue
            ((153, 153, 255), (46/100*self.WINDOW_WIDTH, 7/100*self.WINDOW_HEIGHT)),        # light blue-violet
            ((204, 153, 255), (47.75/100*self.WINDOW_WIDTH, 7/100*self.WINDOW_HEIGHT)),     # light purple
            ((255, 153, 255), (49.5/100*self.WINDOW_WIDTH, 7/100*self.WINDOW_HEIGHT)),      # light magenta
            ((255, 153, 204), (51.25/100*self.WINDOW_WIDTH, 7/100*self.WINDOW_HEIGHT)),     # light pink
            ((204, 204, 204), (53/100*self.WINDOW_WIDTH, 7/100*self.WINDOW_HEIGHT)),        # light gray
        ]

        # sizes for the pencil and the position of the icons
        self.pencil_size_palette = [
            (2, (32/100*self.WINDOW_WIDTH, self.MENU_HEIGHT/2 - self.GENERAL_ICON_SIZE[0]/2)),
            (5, (35/100*self.WINDOW_WIDTH, self.MENU_HEIGHT/2 - self.GENERAL_ICON_SIZE[0]/2)),
            (10, (38/100*self.WINDOW_WIDTH, self.MENU_HEIGHT/2 - self.GENERAL_ICON_SIZE[0]/2)),
            (15, (41/100*self.WINDOW_WIDTH, self.MENU_HEIGHT/2 - self.GENERAL_ICON_SIZE[0]/2)),
            (25, (44/100*self.WINDOW_WIDTH, self.MENU_HEIGHT/2 - self.GENERAL_ICON_SIZE[0]/2)),
        ]

        # create icon surface from import
        self.pencil_icon_surface = self.load_scale_icon_image('Pencil Image.png')
        self.eraser_icon_surface = self.load_scale_icon_image('Eraser Image.png')
        self.color_icon_surface = self.load_scale_icon_image('Colors Image.png')
        self.pencil_size_icon_surface = self.load_scale_icon_image('Pencil Size Image.png')
        self.undo_icon_surface = self.load_scale_icon_image('Undo Image.png')
        self.redo_icon_surface = self.load_scale_icon_image('Redo Image.png')
        self.undo_icon_gray_surface = self.load_scale_icon_image('Undo Image GRAY.png')
        self.redo_icon_gray_surface = self.load_scale_icon_image('Redo Image GRAY.png')
        self.zoom_in_surface = self.load_scale_icon_image('Zoom In Image.png')
        self.zoom_out_surface = self.load_scale_icon_image('Zoom Out Image.png')
        self.zoom_in_gray_surface = self.load_scale_icon_image('Zoom In Image Gray.png')
        self.zoom_out_gray_surface = self.load_scale_icon_image('Zoom Out Image Gray.png')
        self.move_tool_surface = self.load_scale_icon_image('Move Tool Image.png')
        self.save_icon_surface = self.load_scale_icon_image('Save and Startscreen Image.png')
        self.documentation_icon_surface = self.load_scale_icon_image('Documentation Icon Image.png')

        self.size_xs_surface = self.load_scale_icon_image('Size XS Image.png')
        self.size_s_surface = self.load_scale_icon_image('Size S Image.png')
        self.size_m_surface = self.load_scale_icon_image('Size M Image.png')
        self.size_l_surface = self.load_scale_icon_image('Size L Image.png')
        self.size_xl_surface = self.load_scale_icon_image('Size XL Image.png')

        self.pencil_size_list = [self.size_xs_surface, self.size_s_surface,
                                 self.size_m_surface, self.size_l_surface, self.size_xl_surface]

        # create Palette class objects
        self.color_palette_instance = Palette(
            self.main_screen, self.color_palette, "color", self.pencil, self.WINDOW_WIDTH)
        self.pencil_size_palette_instance = Palette(self.main_screen,
                                                    self.pencil_size_palette, "size", self.pencil, self.WINDOW_WIDTH, self.pencil_size_list)

        # create tool rectangles
        self.pencil_icon_rect = self.pencil_icon_surface.get_frect(
            topleft=self.pencil_icon_position)
        self.eraser_icon_rect = self.eraser_icon_surface.get_frect(
            topleft=self.eraser_icon_position)
        self.move_tool_rect = self.move_tool_surface.get_frect(
            topleft=self.move_tool_icon_position)

        # create buttons
        self.color_button = IconButton(self.undo_manager, self.camera,
                                       self.color_icon_surface, self.color_icon_position, self.color_palette_instance)
        self.pencil_size_button = IconButton(self.undo_manager, self.camera, self.pencil_size_icon_surface,
                                             self.pencil_size_icon_position, self.pencil_size_palette_instance)
        self.undo_icon_button = IconButton(self.undo_manager, self.camera, self.undo_icon_surface, self.undo_icon_position,
                                           image_surface_disabled=self.undo_icon_gray_surface, type="undo")
        self.redo_icon_button = IconButton(self.undo_manager, self.camera, self.redo_icon_surface, self.redo_icon_position,
                                           image_surface_disabled=self.redo_icon_gray_surface, type="redo")
        self.zoom_in_button = IconButton(self.undo_manager, self.camera, self.zoom_in_surface, self.zoom_in_icon_position,
                                         image_surface_disabled=self.zoom_in_gray_surface, type="zoom in")
        self.zoom_out_button = IconButton(self.undo_manager, self.camera, self.zoom_out_surface, self.zoom_out_icon_position,
                                          image_surface_disabled=self.zoom_out_gray_surface, type="zoom out")
        self.move_tool_button = IconButton(self.undo_manager, self.camera,
                                           self.move_tool_surface, self.move_tool_icon_position)
        self.save_icon_button = IconButton(self.undo_manager, self.camera,
                                           self.save_icon_surface, self.save_icon_position)
        self.documentation_button = IconButton(self.undo_manager, self.camera,
                                               self.documentation_icon_surface, self.documentation_icon_position)

        # create clock object
        self.clock = pygame.time.Clock()
        self.popup_clock = pygame.time.Clock()

        # create Startscreen
        self.start_screen = StartScreenDocumentation(self.main_screen, os.path.join(
            'src', 'pixelbot_tablet', 'images', 'Welcome Image2.png'),
            os.path.join('src', 'pixelbot_tablet',
                         'images', 'Start Image.png'),
            os.path.join('src', 'pixelbot_tablet',
                         'images', 'Exit Button Image.png'),
            os.path.join('src', 'pixelbot_tablet',
                         'images', 'Background Images.png'),
            self.WINDOW_HEIGHT, self.WINDOW_WIDTH, self.popup_clock, self.undo_manager, self.camera, self.save_manager, self.greeting_message_publisher)

        # start drawing app
        self.run_main_loop()

    # load image from filename and scale it
    def load_scale_icon_image(self, filename):
        file_path = os.path.join('src', 'pixelbot_tablet', 'images', filename)
        return pygame.transform.smoothscale(pygame.image.load(file_path).convert_alpha(), self.GENERAL_ICON_SIZE)

    # reset attributes for new child to start new drawing session
    def reset(self):
        self.undo_manager.reset()
        self.active_tool = self.pencil
        self.pencil.active = False
        self.pencil.last_position = None
        self.pencil.last_add_point_time = None
        self.pencil.color = BLACK
        self.pencil.tool_size = 10
        self.camera.reset()
        self.current_stroke = None
        self.activity_started = False
        self.net_erased_pixels = 0

        # Reset of the variables in the other nodes
        reset_msg = Empty()
        self.reset_publisher.publish(reset_msg)



    def send_drawing_to_vlm(self):
        if self.activity_started:
            # save drawing surface as opencv image
            display_surface = self.drawing_surface.copy()
            
            # Scale the image down to decrease the vlm processing time
            display_surface_size = display_surface.get_size()
            scaled_down_size = tuple(s/4 for s in display_surface_size)
            scaled_down_display_surface = pygame.transform.scale(display_surface, scaled_down_size)

            img_array = np.array(pygame.surfarray.pixels3d(scaled_down_display_surface))

            image_object = np.transpose(img_array, (1, 0, 2))
            image_object = cv2.cvtColor(image_object, cv2.COLOR_RGB2BGR)

            # publish images
            ros_image = self.cv_bridge.cv2_to_imgmsg(image_object)
            self.drawing_publisher.publish(ros_image)

    # start drawing: let user draw by getting user input and display UI
    def run_game(self):

        #self.reset()
        self.drawing_surface.fill(WHITE)
        self.undo_manager.append_drawing(self.drawing_surface)
        self.camera.drawing_version += 1
        last_debug_time = time.time()
        redraw_canvas = True
        redraw_menu = True

        canvas_rect = pygame.Rect(
            0, self.MENU_HEIGHT + 2, self.WINDOW_WIDTH, self.WINDOW_HEIGHT - self.MENU_HEIGHT)
        menu_rect = pygame.Rect(0, 0, self.WINDOW_WIDTH, self.MENU_HEIGHT)

        while True:
            self.clock.tick(30)
            areas_to_update = []

            # event loop
            event_list = pygame.event.get()
            for event in event_list:

                if event.type in (pygame.MOUSEBUTTONDOWN, pygame.MOUSEBUTTONUP, pygame.MOUSEMOTION):
                    if event.pos[1] >= self.MENU_HEIGHT:
                        redraw_canvas = True
                    else:
                        redraw_menu = True

                if event.type == pygame.MOUSEBUTTONDOWN:

                    mouse_position = pygame.mouse.get_pos()

                    # click pencil
                    if self.pencil_icon_rect.collidepoint(mouse_position):
                        self.active_tool = self.pencil

                    # click eraser
                    if self.eraser_icon_rect.collidepoint(mouse_position):
                        self.active_tool = self.eraser

                    # click move tool
                    if self.move_tool_rect.collidepoint(mouse_position):
                        self.active_tool = self.move_tool

                    # click color palette
                    if self.color_button.check_click(mouse_position):
                        self.pencil_size_button.palette.hide()
                        self.color_button.handle_click()

                    # click pencil size palette
                    if self.pencil_size_button.check_click(mouse_position):
                        self.color_button.palette.hide()
                        self.pencil_size_button.handle_click()

                    # click undo button
                    if self.undo_icon_button.check_click(mouse_position):
                        ink_before_undo = count_ink_pixels(self.drawing_surface)

                        self.undo_manager.undo(self.drawing_surface)
                        self.camera.drawing_version += 1

                        ink_after_undo = count_ink_pixels(self.drawing_surface)
                        number_undone_pixels = ink_before_undo - ink_after_undo
                        self.net_erased_pixels += number_undone_pixels

                        redraw_canvas = True

                    # click redo button
                    if self.redo_icon_button.check_click(mouse_position):
                        ink_before_redo = count_ink_pixels(self.drawing_surface)

                        self.undo_manager.redo(self.drawing_surface)
                        self.camera.drawing_version += 1

                        ink_after_redo = count_ink_pixels(self.drawing_surface)
                        number_redone_pixels = ink_before_redo - ink_after_redo
                        self.net_erased_pixels += number_redone_pixels

                        redraw_canvas = True

                    # click zoom in button
                    if self.zoom_in_button.check_click(mouse_position):
                        self.camera.zoom_in()
                        redraw_canvas = True

                    # click zoom out button
                    if self.zoom_out_button.check_click(mouse_position):
                        self.camera.zoom_out()
                        redraw_canvas = True

                    # check what user chooses in palette
                    self.color_palette_instance.handle_palette_click(
                        mouse_position)
                    self.pencil_size_palette_instance.handle_palette_click(
                        mouse_position)

                elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:

                    # make undo button black
                    redraw_menu = True

                    # if user drew or erased something: capture current drawing
                    mouse_position = pygame.mouse.get_pos()
                    if (self.active_tool == self.pencil or self.active_tool == self.eraser) and mouse_position[1] >= self.MENU_HEIGHT:

                        if not self.undo_manager.compare_drawing_surface_to_last(self.drawing_surface):
                            self.undo_manager.append_drawing(self.drawing_surface)

                        # Pencil tool is released: current stroke is finished, save it
                        # Note: A stroke needs to have at least two points
                        # if pencil tool is released, but there were no movements previously
                        # then current_stroke is none because it is only made of one point
                        # so we don't consider that to be a stroke and we don't save anything
                        if self.active_tool == self.pencil and self.current_stroke:
                            self.undo_manager.append_stroke(self.current_stroke)
                            self.current_stroke = None
                            self.undo_manager.last_drawing_action = DRAW

                        if self.active_tool == self.eraser:
                            self.undo_manager.last_drawing_action = ERASE

                    # click save button
                    if self.save_icon_button.check_click(mouse_position):
                        popup = PopupWindow(
                            self.main_screen, pygame.font.SysFont('arial', 25), self.popup_clock)
                        answer = popup.ask_confirmation(
                            "Save and go back to start?")
                        if answer == "yes":
                            activity_time = time.time() - self.activity_start_time

                            # Deactivate speech recognizer
                            recognizer_state_msg = Bool()
                            recognizer_state_msg.data = False
                            self.recognizer_state_publisher.publish(recognizer_state_msg)

                            # Change the robot listening state to None
                            robot_state_msg = String()
                            robot_state_msg.data = "None"
                            self.display_robot_state_publisher.publish(robot_state_msg)

                            # save all data
                            self.save_manager.save_drawing(self.activity_start_time)
                            clean_stroke_data_stack = self.undo_manager.stroke_data_stack[:self.undo_manager.current_index_stroke_data_stack]
                            undone_erased_stroke_data_array = self.undo_manager.undone_erased_stroke_list + self.undo_manager.stroke_data_stack[self.undo_manager.current_index_stroke_data_stack:]
                            self.save_manager.save_stroke_data(clean_stroke_data_stack, undone_erased_stroke_data_array, activity_time, self.net_erased_pixels)

                            # reset all variables and other nodes
                            self.reset()

                            return

                    # click documentation button
                    if self.documentation_button.check_click(mouse_position):
                        self.start_screen.show_documentation()
                        redraw_canvas = True

                if self.pressure_recording == "enabled":
                    self.current_stroke, number_erased_pixels = self.active_tool.handle_event(
                        event, self.drawing_surface, self.pencil, self.eraser, self.move_tool, self.active_tool, self.MENU_HEIGHT, self.current_stroke, self.activity_start_time, self.pressure_reader.pressure)
                    self.net_erased_pixels += number_erased_pixels
                else:
                    self.current_stroke, number_erased_pixels = self.active_tool.handle_event(
                        event, self.drawing_surface, self.pencil, self.eraser, self.move_tool, self.active_tool, self.MENU_HEIGHT, self.current_stroke, self.activity_start_time)
                    self.net_erased_pixels += number_erased_pixels

            # send last drawing version to vlm node
            if time.time() - last_debug_time >= 5:
                self.send_drawing_to_vlm()
                last_debug_time = time.time()


            if redraw_canvas:
                # display drawing surface and limit offset
                pygame.draw.rect(self.drawing_surface, BLUE,
                                 self.drawing_surface.get_rect(), BLUE_BORDER_THICKNESS)
                self.camera.limit_offset()
                self.camera.custom_display()

                self.eraser.display_eraser_border(
                    self.active_tool, self.eraser, self.MENU_HEIGHT)

                areas_to_update.append(canvas_rect)
                redraw_canvas = False

            if redraw_menu:

                # display interface components
                pygame.draw.rect(self.main_screen, WHITE,
                                 (0, 0, self.DRAWING_SURFACE_WIDTH, self.MENU_HEIGHT))

                pygame.draw.line(self.main_screen, BLACK, (0, self.MENU_HEIGHT),
                                 (self.WINDOW_WIDTH, self.MENU_HEIGHT), 2)

                # display tool icons with fitting border
                self.main_screen.blit(
                    self.pencil_icon_surface, self.pencil_icon_rect)
                self.main_screen.blit(
                    self.eraser_icon_surface, self.eraser_icon_rect)
                self.main_screen.blit(
                    self.move_tool_surface, self.move_tool_rect)
                self.pencil.display_borders_tool(
                    self.pencil_icon_rect, self.active_tool)
                self.eraser.display_borders_tool(
                    self.eraser_icon_rect, self.active_tool)

                self.move_tool.display_borders_tool(
                    self.move_tool_rect, self.active_tool)

                # display buttons
                self.color_button.display(self.main_screen, self.pencil.color)
                self.pencil_size_button.display(self.main_screen, GRAY)
                self.undo_icon_button.display(self.main_screen, GRAY)
                self.redo_icon_button.display(self.main_screen, GRAY)
                self.zoom_in_button.display(self.main_screen, GRAY)
                self.zoom_out_button.display(self.main_screen, GRAY)
                self.save_icon_button.display(self.main_screen, GRAY)
                self.documentation_button.display(self.main_screen, GRAY)

                # display palettes
                self.color_palette_instance.display(
                    self.main_screen, GRAY)
                self.pencil_size_palette_instance.display(
                    self.main_screen, GRAY)

                areas_to_update.append(menu_rect)
                redraw_menu = False

            if areas_to_update:
                pygame.display.update(areas_to_update)

    # run main game loop: run start screen and drawing app till user closes with exit button
    def run_main_loop(self):
        while rclpy.ok():
            keep_continue = self.start_screen.run()
            self.activity_start_time = time.time()
            self.activity_started = True
            if not keep_continue:
                pygame.quit()
                rclpy.shutdown()
                sys.exit()
            self.run_game()


def main(args=None):
    rclpy.init(args=args)
    node = DrawingApplicationNode()
    rclpy.spin(node) # TODO: spin once? pygame clock?
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
