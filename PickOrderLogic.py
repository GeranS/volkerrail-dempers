import numpy as np

import Camera
import ConversionService
import HttpService

import cv2
import time


# todo: still getting an array index out of range exception
def find_first_single(dampers):
    for row in dampers:
        if len(row) == 1 and not row[0].get_moved():
            return row[0]

        if len(row) == 0:
            continue

        i = 0
        while i < len(row):
            if row[i] is not None and not row[i].get_moved():
                return row[i]
            i += 1

    return None


class PickOrderLogic:
    def __init__(self):
        self.busy = False

        self.camera = Camera.Camera()
        self.http_service = HttpService.HttpService(self)
        self.conversion_service = ConversionService.ConversionService.get_instance()

        # tells the robot to move to a safe position
        time.sleep(1)
        self.http_service.send_command('SAFE')

    # Automatic mode automatically finds the dampers and moves them, without waiting for user input
    def start_automatic_mode(self):
        cv2.namedWindow('dempers')

        while True:
            key = cv2.waitKey(1)

            # and self.busy is False
            if key == ord('s'):
                break

        # Every iteration is one layer
        while True:
            image, detection_z, layer_z = self.camera.get_top_layer_image()
            slats = self.camera.find_slats(image, detection_z)

            if slats is not None:
                self.remove_slats(slats, layer_z)

                image, detection_z, layer_z = self.camera.get_top_layer_image()

            array_of_dampers, original_image = self.camera.find_dampers(image, detection_z, layer_z)
            image = original_image.copy()

            while True:
                if array_of_dampers is None:
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

                for row in array_of_dampers:
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
                layer_done = self.choose_next_pick(array_of_dampers)

                if layer_done:
                    break

    def choose_next_pick(self, dampers):
        while self.busy:
            time.sleep(0.5)

        damper_1, damper_2 = self.find_first_pair(dampers)

        # todo: single damper detection/pickup and movement needs a left/right system
        if damper_1 is None:
            damper_single = find_first_single(dampers)

            if damper_single is None:
                return True

            damper_single.set_moved()

            target_x, target_y, target_z = self.conversion_service.convert_to_robot_coordinates(damper_single.get_x(),
                                                                                                damper_single.get_y(),
                                                                                                damper_single.get_z())

            # offset because it's a single damper
            # todo: check offset
            target_y = target_y - 0.05

            self.http_service.send_move_command(target_x, target_y, target_z)
            return False
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

            self.http_service.send_move_command(target_x, target_y, target_z)
            return False

        return True

    def find_first_pair(self, dampers):
        for column in dampers:
            counter = 0
            while True:
                if column[counter] is not None and column[counter + 1] is not None:
                    if column[counter].get_moved() is False and column[counter + 1].get_moved() is False:
                        return column[counter], column[counter + 1]
                counter += 2
                if counter >= len(column):
                    break

        return None, None

    def remove_slats(self, slats, layer_z):

        for slat in slats:
            x, y = slat
            robot_x, robot_y, robot_z = self.conversion_service.convert_to_robot_coordinates(x, y, layer_z)

            while self.busy:
                time.sleep(0.5)

            self.http_service.send_move_slats_command(robot_x, robot_y, robot_z)

    def set_robot_done(self):
        self.busy = False
