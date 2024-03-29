import Camera
import ConversionService
import HttpService
from RailDetection import RailCamera

import cv2
import time
import numpy as np
import _thread


def find_first_single(dampers):
    for row in dampers:
        if len(row) == 1 and not row[0].moved:
            return row[0], 2

        if len(row) == 0:
            continue

        i = 0
        while i < len(row):
            if row[i] is not None and not row[i].moved:
                if i % 2 == 0:
                    return row[i], 2
                else:
                    return row[i], 1
            i += 1

    return None, None


class PickOrderLogic:
    def __init__(self):
        self.busy = False
        self.auto = False
        self.paused = False
        self.place_next = False
        self.shutdown = False
        self.paused_sent_safe = False
        self.image = np.zeros(shape=[480, 640, 3], dtype=np.uint8)

        self.amount_of_dampers_previous_layer = 0

        # Current layer of dampers, take new picture if none or all moved
        self.dampers = []
        # Slats
        self.slats = []
        # layer z
        self.layer_z = []

        self.camera = Camera.Camera()
        self.http_service = HttpService.HttpService(self)
        self.conversion_service = ConversionService.ConversionService.get_instance()

        RailCamera.RailCamera(self.http_service)

    def start_automatic_mode(self):
        self.auto = True

        while self.auto:

            time.sleep(0.1)

            if self.shutdown:
                self.http_service.send_shutdown_command()
                break

            if self.paused:
                if self.paused_sent_safe is False:
                    self.http_service.send_safe_command()
                    self.paused_sent_safe = True

            # Wait for the robot to be free and the program to unpause
            if self.busy or self.paused:
                continue

            self.paused_sent_safe = False

            if self.check_if_all_dampers_have_been_moved():
                self.dampers = []

            if len(self.slats) != 0:
                print("Removing slats...")
                self.remove_slats()
                self.slats = []
            elif len(self.slats) == 0 and len(self.dampers) == 0:
                self.http_service.send_picture_command()
                self.busy = True

                while self.busy:
                    time.sleep(0.5)

                detection_z, layer_z = self.camera.get_top_layer_image()

                if layer_z is None:
                    print("Could not find matching layer_z, check pallet.")
                    self.paused = True
                    self.http_service.send_code_to_plc(0)  # Error
                    self.slats = []
                    self.dampers = []
                    continue

                slats = self.camera.find_slats(layer_z)

                if slats is None:
                    print("No slats found. Finding dampers.")
                    dampers = self.camera.find_dampers(detection_z, layer_z)

                    if dampers is None:
                        print("Could not find dampers.")
                        self.paused = True
                        if self.layer_z == 0.77:
                            self.http_service.send_code_to_plc(1)  # Out of dampers
                        else:
                            self.http_service.send_code_to_plc(0)  # Error
                        continue

                    self.dampers = dampers
                else:
                    self.slats = slats
            elif self.place_next:
                print("Placing damper.")
                self.choose_next_pick()
                self.place_next = False

        self.auto = False


    # Mode for testing
    def start_testing_mode(self):

        while True:
            detection_z, layer_z = self.camera.get_top_layer_image()
            slats = self.camera.find_slats(layer_z)

            if slats is None:
                self.camera.find_dampers(detection_z, layer_z)

    # todo: Not necessarily anything wrong with this method in general, but it looks ugly; please fix.
    def choose_next_pick(self):
        self.busy = True

        damper_pair = self.find_first_pair()

        if damper_pair is None:
            damper_single, grab_mode = find_first_single(self.dampers)

            if damper_single is None:
                self.busy = False
                self.dampers = []
                return True

            damper_single.moved = True

            target_x, target_y, target_z = self.conversion_service.convert_to_robot_coordinates(damper_single.x,
                                                                                                damper_single.y,
                                                                                                damper_single.z)

            # offset because it's a single damper
            if grab_mode == 1:
                target_y = target_y + 0.05
            elif grab_mode == 2:
                target_y = target_y - 0.05

            self.http_service.send_move_command(target_x, target_y, target_z, grab_mode)
            return
        elif damper_pair is not None:
            damper_1, damper_2 = damper_pair

            damper_1.moved = True
            damper_2.moved = True

            damper_1_x, damper_1_y, damper_1_z = self.conversion_service.convert_to_robot_coordinates(damper_1.x,
                                                                                                      damper_1.y,
                                                                                                      damper_1.z)

            damper_2_x, damper_2_y, _ = self.conversion_service.convert_to_robot_coordinates(damper_2.x,
                                                                                             damper_2.y,
                                                                                             damper_2.z)

            target_x = (damper_1_x + damper_2_x) / 2
            target_y = (damper_1_y + damper_2_y) / 2
            target_z = damper_1_z

            self.http_service.send_move_command(target_x, target_y, target_z, 0)
            return

        self.busy = False
        self.dampers = []

    def check_if_all_dampers_have_been_moved(self):
        for column in self.dampers:
            for damper in column:
                if damper is not None and damper.moved is False:
                    return False
        return True

    def find_first_pair(self):
        for column in self.dampers:
            if len(column) == 1:
                continue
            counter = 0
            while True:
                if column[counter] is not None and column[counter + 1] is not None:
                    if column[counter].moved is False and column[counter + 1].moved is False:
                        return (column[counter], column[counter + 1])
                counter += 2
                if counter >= len(column) - 1:
                    break

        return None

    def remove_slats(self):
        for slat in self.slats:
            if slat.moved is False:
                x, y, z = slat.x, slat.y, slat.z
                robot_x, robot_y, robot_z = self.conversion_service.convert_to_robot_coordinates(x, y, z)

                robot_x = robot_x - 0.045

                while self.busy:
                    time.sleep(0.5)

                self.busy = True
                self.http_service.send_move_slats_command(robot_x, robot_y, robot_z)
                slat.moved = True

        self.slats = []

        while self.busy:
            time.sleep(0.5)

        self.http_service.send_picture_command()

        self.busy = True

        while self.busy:
            time.sleep(0.5)

    def set_robot_done(self):
        self.busy = False

    def pause_program(self):
        self.paused = True

    def unpause_program(self):
        self.paused = False

    def shutdown(self):
        self.shutdown = True
