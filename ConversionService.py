import json


class ConversionService:
    __instance = None

    @staticmethod
    def get_instance():
        if ConversionService.__instance is None:
            ConversionService()
        return ConversionService.__instance

    def __init__(self):
        if ConversionService.__instance is not None:
            raise Exception("This class is a singleton")
        else:
            ConversionService.__instance = self

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

            self.quarter_turn = calibration_json['quarter_turn']

            self.x_inversion = calibration_json['x_inversion']
            self.y_inversion = calibration_json['y_inversion']

            self.layer_heights = calibration_json['layer_heights']

    def reload_calibration_file(self):
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

        self.quarter_turn = calibration_json['quarter_turn']

        self.x_inversion = calibration_json['x_inversion']
        self.y_inversion = calibration_json['y_inversion']

        self.layer_heights = calibration_json['layer_heights']

    # converts pixels at a given distance to meters
    # makes use of the calibration data
    # todo: check if pixel to meter conversion is different near edges of the view
    def convert_pixels_to_meters(self, pixels, z):
        meters_per_pixel = self.calibration_object_length_in_meters / self.calibration_object_length_in_pixels
        relative_distance_difference = z / self.calibration_object_distance_in_meters
        meters_per_pixel = meters_per_pixel * relative_distance_difference
        meters = pixels * meters_per_pixel

        return meters

    def convert_meters_to_pixels(self, meters, z):
        pixels_per_meter = self.calibration_object_length_in_pixels / self.calibration_object_length_in_meters
        relative_distance_difference = z / self.calibration_object_distance_in_meters
        pixels_per_meter = pixels_per_meter * relative_distance_difference
        pixels = int(meters * pixels_per_meter)

        return pixels

    def scale_pixel_area(self, pixel_area, current_z, new_z):
        relative = current_z / new_z
        new_area = pixel_area * (relative ** 2)
        return new_area

    # converts from camera pixel coordinates to robot coordinates
    # makes use of the calibration data
    # todo: implement quarter turn, x inversion, and y inversion from calibration.json
    def convert_to_robot_coordinates(self, x, y, z):
        self.reload_calibration_file()

        calibration_coordinates_camera = (
            self.convert_pixels_to_meters(self.calibration_pixel_coordinates_camera[0], z),
            self.convert_pixels_to_meters(-self.calibration_pixel_coordinates_camera[1], z),
            self.calibration_pixel_coordinates_camera[2])

        x_difference = calibration_coordinates_camera[0] - self.calibration_coordinates_robot[0]
        y_difference = calibration_coordinates_camera[1] - self.calibration_coordinates_robot[1]
        # different because the z axis on the camera is inverse to the z axis on the robot
        camera_height = calibration_coordinates_camera[2] + self.calibration_coordinates_robot[2]
        new_z = camera_height - z

        x_meters = self.convert_pixels_to_meters(x, z)
        y_meters = self.convert_pixels_to_meters(-y, z)

        new_x = x_meters - x_difference
        new_y = y_meters - y_difference

        return new_x, new_y, new_z

    def get_layer_z(self, detection_z):
        best_layer_height = 0
        difference = 100.0

        for layer in self.layer_heights:
            if abs(detection_z - layer) < difference:
                best_layer_height = layer
                difference = abs(detection_z - layer)

        if difference > 0.04:
            return None

        return best_layer_height
