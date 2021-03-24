import Camera as cam
import HttpService as http

import cv2
import json
import time


class PickOrderLogic:
    def __init__(self):
        self.busy = True

        self.camera = cam.Camera()
        self.http_service = http.HttpService(self)

        with open("volkerrail-dempers/calibration.json", "r") as c:
            calibration_json = json.load(c)
        c.close()

        self.calibration_object_length_in_meters = calibration_json['calibration_object_length_in_meters']
        self.calibration_object_length_in_pixels = calibration_json['calibration_object_length_in_pixels']
        self.calibration_object_distance_in_meters = calibration_json['calibration_object_distance_in_meters']
        # (x, y, z)
        self.calibration_coordinates_robot = calibration_json['calibration_coordinates_robot']
        # (x, y, z)
        self.calibration_pixel_coordinates_camera = calibration_json['calibration_pixel_coordinates_camera']

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
            image, z = self.camera.get_top_layer_image()
            array_of_dampers, original_image = self.camera.find_dampers(image, z)

            image = original_image.copy()

            while True:
                damper_count = 1

                for row in array_of_dampers:
                    for damper in row:
                        if damper is not None:
                            if damper.get_moved():
                                cv2.putText(image, str(damper_count) + " moved", (damper.get_x(), damper.get_y()), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                            (255, 0, 0))
                            else:
                                cv2.putText(image, str(damper_count), (damper.get_x(), damper.get_y()), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                            (255, 0, 0))
                            damper_count += 1

                cv2.imshow('dempers', image)

                while True:
                    key = cv2.waitKey(1)

                    # and self.busy is False
                    if key == ord('s'):
                        break

                cv2.destroyAllWindows()
                layer_done = self.choose_next_pick(array_of_dampers)

                if layer_done:
                    break

    def choose_next_pick(self, dampers):
        damper_1, damper_2 = self.find_first_pair(dampers)

        if damper_1 is None:
            damper_single = self.find_first_single(dampers)

            if damper_single is None:
                return True

            damper_single.set_moved()

            target_x, target_y, target_z = self.convert_to_robot_coordinates(damper_single.get_x(),
                                                                        damper_single.get_y(), damper_single.get_z())

            # offset because it's a single damper
            target_coordinates = target_x - 0.05, target_y, target_z

            command_string = 'DAMPER[ ]({})'.format(target_coordinates)
            self.http_service.send_command(command_string)
            return False
        elif damper_1 is not None and damper_2 is not None:
            command_string = '{},{} : {},{}'.format(self.convert_pixels_to_meters(damper_1.get_x(), damper_1.get_z()),
                                                    self.convert_pixels_to_meters(damper_1.get_y(), damper_1.get_z()),
                                                    self.convert_pixels_to_meters(damper_2.get_x(), damper_1.get_z()),
                                                    self.convert_pixels_to_meters(damper_2.get_y(), damper_1.get_z()))
            damper_1.set_moved()
            damper_2.set_moved()

            damper_1_x, damper_1_y, damper_1_z = self.convert_to_robot_coordinates(damper_1.get_x(), damper_1.get_y(), damper_1.get_z())

            self.http_service.send_command(command_string)
            return False

        return True

    def find_first_single(self, dampers):
        for row in dampers:
            if len(row) == 1 and not row[0].get_moved():
                return row[0]

            if len(row) == 0:
                continue

            i = 0
            while i < len(row):
                if row[i] is not None and not row[i].get_moved():
                    return row[i + 1]
                i += 2

        return None

    def find_first_pair(self, dampers):
        for row in dampers:
            i = 0
            while i < len(row) - 1:
                if row[i] is not None and row[i + 1] is not None and not row[i].get_moved() and not row[i + 1].get_moved():
                    return row[i], row[i + 1]
                i += 2

        return None, None

    # converts pixels at a given distance to meters
    # makes use of the calibration data
    def convert_pixels_to_meters(self, pixels, z):
        meters_per_pixel = self.calibration_object_length_in_meters / self.calibration_object_length_in_pixels
        relative_distance_difference = z / self.calibration_object_distance_in_meters
        meters_per_pixel = meters_per_pixel * relative_distance_difference
        meters = pixels * meters_per_pixel

        return meters

    # converts from camera pixel coordinates to robot coordinates
    # makes use of the calibration data
    def convert_to_robot_coordinates(self, x, y, z):
        calibration_coordinates_camera = (self.convert_pixels_to_meters(self.calibration_pixel_coordinates_camera[0], z),
                                          self.convert_pixels_to_meters(self.calibration_pixel_coordinates_camera[1], z),
                                          self.calibration_pixel_coordinates_camera[2])

        x_meters = self.convert_pixels_to_meters(x, z)
        y_meters = self.convert_pixels_to_meters(y, z)

        x_difference = calibration_coordinates_camera[0] - self.calibration_coordinates_robot[0]
        y_difference = calibration_coordinates_camera[1] - self.calibration_coordinates_robot[1]
        # different because the z axis on the camera is inverse to the z axis on the robot
        camera_height = calibration_coordinates_camera[2] + self.calibration_coordinates_robot[2]
        new_z = camera_height - z

        new_x = x_meters + x_difference
        new_y = y_meters + y_difference

        return new_x, new_y, new_z

    def set_robot_done(self):
        self.busy = False
