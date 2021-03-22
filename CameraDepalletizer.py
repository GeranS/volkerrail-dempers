import cv2
import numpy as np
import pyrealsense2 as rs

layer_thresholds = {0: 0.47, 1: 0.4}


class Damper:
    def __init__(self, x, y, z, moved):
        self.x = x
        self.y = y
        self.z = z
        self.moved = moved

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_z(self):
        return self.z

    def get_moved(self):
        return self.moved

    def set_moved(self):
        self.moved = True

class CameraState:
    def __init__(self, *args, **kwargs):
        self.distance = 0.47
        self.paused = False
        # level of the pallet, zero is top layer
        self.pallet_level = 0

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()

        self.threshold_filter = rs.threshold_filter()
        self.threshold_filter.set_option(rs.option.max_distance, self.distance)

        config.enable_stream(rs.stream.depth, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(config)
        self.profile = self.pipeline.get_active_profile()

        self.sensor = self.profile.get_device().first_depth_sensor()
        self.sensor.set_option(rs.option.visual_preset, value=1)

        self.colorizer = rs.colorizer(3)

        self.colorizer.set_option(rs.option.min_distance, 0)
        self.colorizer.set_option(rs.option.max_distance, 0.1)
        self.colorizer.set_option(rs.option.histogram_equalization_enabled, False)

    def get_dampers(self):
        layer = self.determine_top_layer()

        # Set threshold filter to appropriate distance
        self.threshold_filter.set_option(rs.option.max_distance, layer_thresholds[layer])

        frames = self.pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        depth_frame = self.threshold_filter.process(depth_frame)

        depth_colormap = np.asanyarray(self.colorizer.colorize(depth_frame).get_data())

        array_of_dampers = self.find_dampers_in_image(depth_colormap)

        return array_of_dampers

    def determine_top_layer(self):
        # Increase threshold to 3 meters
        self.threshold_filter.set_option(rs.option.max_distance, 3)

        # Get frame
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        depth_frame = self.threshold_filter.process(depth_frame)
        depth_image = np.asanyarray(depth_frame.get_data())

        # Might not be reliable because of errors
        closest_object_in_meters = np.amin(depth_image)/0.001

        closest_layer = 0
        last_closest_distance = 1000

        i = 0
        for layer in layer_thresholds:
            if abs(layer[i] - closest_object_in_meters) < last_closest_distance:
                last_closest_distance = abs(layer[i] - closest_object_in_meters)
                closest_layer = i

        return closest_layer

    def find_dampers_in_image(self, colormap):
        gray = cv2.bilateralFilter(colormap, 11, 17, 17)

        iterations = 10

        kernel = np.ones((5, 5), np.uint8)
        erosion = cv2.erode(gray, kernel, iterations=iterations)
        kernel = np.ones((4, 4), np.uint8)
        dilation = cv2.dilate(erosion, kernel, iterations=iterations)

        edged = cv2.Canny(dilation, 30, 200)

        contours, _ = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)

            if area < 20000:
                continue

            m = cv2.moments(contour)

            x = 0
            y = 0

            if m['m00'] != 0.0:
                cX = int(m['m10'] / m['m00'])
                cY = int(m['m01'] / m['m00'])

            x, y, w, h = cv2.boundingRect(contour)

        # todo: put all the found dampers in an array
