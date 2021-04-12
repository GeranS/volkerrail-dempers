import Camera
import ConversionService
import HttpService

import cv2
import time
import numpy as np


def find_first_single(dampers):
    for row in dampers:
        if len(row) == 1 and not row[0].get_moved():
            return row[0], 2

        if len(row) == 0:
            continue

        i = 0
        while i < len(row):
            if row[i] is not None and not row[i].get_moved():
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

    def start_automatic_mode(self):
        self.auto = True

        while self.auto:

            time.sleep(0.1)

            if self.shutdown:
                self.http_service.send_shutdown_command()
                break

            if self.paused:
                self.http_service.send_safe_command()

            # Wait for the robot to be free and the program to unpause
            if self.busy or self.paused:
                print('Busy or paused.')
                continue

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
                    self.http_service.send_code_to_plc(0)
                    continue

                slats = self.camera.find_slats(layer_z)

                if slats is None:
                    print("No slats found. Finding dampers.")
                    dampers, image = self.camera.find_dampers(detection_z, layer_z)
                    self.dampers = dampers

                    if dampers is None:
                        print("Could not find dampers.")
                        self.paused = True
                        self.http_service.send_code_to_plc(0)
                        continue
                else:
                    self.slats = slats
            elif self.place_next:
                print("Placing damper.")
                self.choose_next_pick()
                self.place_next = False

        self.auto = False


    # Mode for testing
    def start_testing_mode(self):

        if self.auto:
            return

        cv2.namedWindow('dempers')

        while True:
            key = cv2.waitKey(1)

            # and self.busy is False
            if key == ord('s'):
                break

        # Every iteration is one layer
        while True:
            self.http_service.send_picture_command()
            self.busy = True

            while self.busy:
                time.sleep(0.5)

            image, detection_z, layer_z = self.camera.get_top_layer_image()

            if image is None or detection_z is None or layer_z is None:
                blank_image = np.zeros(shape=[480, 640, 3], dtype=np.uint8)
                cv2.putText(blank_image, "No dampers found, please check pallet.", (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50, 255, 50))

                while True:
                    cv2.imshow('dempers', blank_image)
                    key = cv2.waitKey(1)

                    # and self.busy is False
                    if key == ord('s'):
                        break
                continue

            self.layer_z = layer_z

            self.slats = self.camera.find_slats(self.layer_z)

            if self.slats is not None:
                self.remove_slats()

                image, detection_z, self.layer_z = self.camera.get_top_layer_image()

            self.dampers, original_image = self.camera.find_dampers(image, detection_z, self.layer_z)
            image = original_image.copy()

            while True:
                if self.dampers is None:
                    cv2.putText(original_image, "No dampers found, please check pallet.", (10, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50, 255, 50))
                    cv2.imshow('dempers', original_image)

                    while True:
                        key = cv2.waitKey(1)

                        # and self.busy is False
                        if key == ord('s'):
                            break

                    break

                damper_count = 1

                for row in self.dampers:
                    for damper in row:
                        if damper is not None:
                            if damper.get_moved():
                                cv2.putText(image, str(damper_count) + " moved", (damper.get_x(), damper.get_y()),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                            (255, 0, 0))
                            else:
                                cv2.putText(image, str(damper_count), (damper.get_x(), damper.get_y()),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                            (255, 0, 0))
                            damper_count += 1

                cv2.imshow('dempers', image)

                while True:
                    key = cv2.waitKey(1)

                    # and self.busy is False
                    if key == ord('s'):
                        break

                while self.busy:
                    time.sleep(0.5)
                    self.busy = True
                layer_done = self.choose_next_pick()

                if layer_done:
                    break

    # todo: Not necessarily anything wrong with this method in general, but it looks ugly; please fix.
    def choose_next_pick(self):
        self.busy = True

        damper_1, damper_2 = self.find_first_pair()

        if damper_1 is None:
            damper_single, grab_mode = find_first_single(self.dampers)

            if damper_single is None:
                self.busy = False
                self.dampers = []
                return True

            damper_single.set_moved()

            target_x, target_y, target_z = self.conversion_service.convert_to_robot_coordinates(damper_single.get_x(),
                                                                                                damper_single.get_y(),
                                                                                                damper_single.get_z())

            # offset because it's a single damper
            if grab_mode == 1:
                target_y = target_y + 0.05
            elif grab_mode == 2:
                target_y = target_y - 0.05

            self.http_service.send_move_command(target_x, target_y, target_z, grab_mode)
            return
        elif damper_1 is not None and damper_2 is not None:
            damper_1.set_moved()
            damper_2.set_moved()

            damper_1_x, damper_1_y, damper_1_z = self.conversion_service.convert_to_robot_coordinates(damper_1.get_x(),
                                                                                                      damper_1.get_y(),
                                                                                                      damper_1.get_z())

            damper_2_x, damper_2_y, _ = self.conversion_service.convert_to_robot_coordinates(damper_2.get_x(),
                                                                                             damper_2.get_y(),
                                                                                             damper_2.get_z())

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
                if damper.get_moved() is False:
                    return False
        return True

    def find_first_pair(self):
        for column in self.dampers:
            if len(column) == 1:
                continue
            counter = 0
            while True:
                if column[counter] is not None and column[counter + 1] is not None:
                    if column[counter].get_moved() is False and column[counter + 1].get_moved() is False:
                        return column[counter], column[counter + 1]
                counter += 2
                if counter >= len(column) - 1:
                    break

        return None, None

    def remove_slats(self):
        for slat in self.slats:
            if slat.moved is False:
                x, y, z = slat.x, slat.y, slat.z
                robot_x, robot_y, robot_z = self.conversion_service.convert_to_robot_coordinates(x, y, z)

                robot_x = robot_x - 0.05

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
