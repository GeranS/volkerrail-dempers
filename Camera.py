import collections

import cv2
import numpy as np
import pyrealsense2 as rs
import json

from ConversionService import ConversionService


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


class Camera:
    def __init__(self):
        self.conversion_service = ConversionService.get_instance()

        self.pipeline = rs.pipeline()
        config = rs.config()

        self.threshold_filter = rs.threshold_filter()
        self.threshold_filter.set_option(rs.option.max_distance, 4)

        config.enable_stream(rs.stream.depth, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(config)
        self.profile = self.pipeline.get_active_profile()

        self.sensor = self.profile.get_device().first_depth_sensor()
        self.sensor.set_option(rs.option.visual_preset, value=1)
        self.sensor.set_option(rs.option.motion_range, value=200)

        # Set colour palette to white to black
        self.colorizer = rs.colorizer(3)

        self.colorizer.set_option(rs.option.min_distance, 0)
        self.colorizer.set_option(rs.option.max_distance, 0.1)
        self.colorizer.set_option(rs.option.histogram_equalization_enabled, False)

    def get_top_layer_image(self):
        frames = self.pipeline.wait_for_frames()

        # Set distance to 4 meters so it can see the whole pallet
        self.threshold_filter.set_option(rs.option.max_distance, 4)

        depth_frame = frames.get_depth_frame()
        depth_frame = self.threshold_filter.process(depth_frame)

        depth_image = np.asanyarray(depth_frame.get_data())
        depth_image = depth_image[80:320, 80:480]
        depth_image = depth_image[depth_image != 0]

        counter = collections.Counter(depth_image)
        # todo: determine most reliable most common set size
        most_common = counter.most_common(500)

        print(most_common)

        smallest_most_common = 10000000

        for m in most_common:
            if m[0] < smallest_most_common:
                smallest_most_common = m[0]

        # todo: Works for now, but requires further tuning for reliability
        closest_object_in_meters = float(smallest_most_common) * self.sensor.get_option(rs.option.depth_units)
        distance_top_layer = closest_object_in_meters + 0.03
        print(distance_top_layer)

        # Set appropriate distance for layer
        self.threshold_filter.set_option(rs.option.max_distance, distance_top_layer)

        frames = self.pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        depth_frame = self.threshold_filter.process(depth_frame)

        depth_colormap = np.asanyarray(self.colorizer.colorize(depth_frame).get_data())

        return depth_colormap, distance_top_layer

    # Finds the dampers in the provided white to black image
    def find_dampers(self, image, z):
        gray = cv2.bilateralFilter(image, 11, 17, 17)

        print("z is: " + str(z))

        cv2.destroyAllWindows()

        iterations = 10
        kernel = np.ones((5, 5), np.uint8)
        erosion = cv2.erode(gray, kernel, iterations=iterations)
        kernel = np.ones((5, 5), np.uint8)
        dilation = cv2.dilate(erosion, kernel, iterations=iterations)

        cv2.destroyAllWindows()

        edged = cv2.Canny(dilation, 30, 200)

        while True:
            cv2.imshow('dempers', dilation)

            key = cv2.waitKey(1)

            # and self.busy is False
            if key == ord('s'):
                break

        contours, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        array_of_damper_locations = []

        for contour in contours:
            cv2.drawContours(image, [contour], 0, (0, 0, 255), 2)
            area = cv2.contourArea(contour)

            if area < 3000:
                continue

            m = cv2.moments(contour)

            # More accurate than the bounding box, especially for single damper pickups
            if m['m00'] != 0.0:
                c_x = int(m['m10'] / m['m00'])
                c_y = int(m['m01'] / m['m00'])

                x, y, w, h = cv2.boundingRect(contour)

                print(w/h)

                if 1.9 < w/h < 2.4:
                    # todo: this needs to be adjusted to be slightly bigger than the area of a damper at the given
                    #  distance
                    if area > 10000:
                        # todo: split up a possible 3 or 4 big damper
                        damper_1_2_x = x
                        damper_3_4_x = x + int((w/2))

                        damper_1_3_y = y
                        damper_2_4_y = y + int((h/2))

                        array_of_damper_locations.append(Damper(damper_1_2_x, damper_1_3_y, z, False))
                        array_of_damper_locations.append(Damper(damper_1_2_x, damper_2_4_y, z, False))
                        array_of_damper_locations.append(Damper(damper_3_4_x, damper_1_3_y, z, False))
                        array_of_damper_locations.append(Damper(damper_3_4_x, damper_2_4_y, z, False))

                        cv2.rectangle(image, (damper_1_2_x, damper_1_3_y), (damper_1_2_x + int(w / 2), damper_1_3_y + int(h/2)),
                                      (255, 0, 0),
                                      1)
                        cv2.rectangle(image, (damper_1_2_x, damper_2_4_y), (damper_1_2_x + int(w / 2), damper_2_4_y + int(h/2)),
                                      (255, 0, 0),
                                      1)
                        cv2.rectangle(image, (damper_3_4_x, damper_1_3_y), (damper_3_4_x + int(w / 2), damper_1_3_y + int(h/2)),
                                      (255, 0, 0),
                                      1)
                        cv2.rectangle(image, (damper_3_4_x, damper_2_4_y), (damper_3_4_x + int(w / 2), damper_2_4_y + int(h/2)),
                                      (255, 0, 0),
                                      1)
                    else:
                        print("damper added")
                        array_of_damper_locations.append(Damper(c_x, c_y, z, False))
                        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 1)
                elif 4.2 < w/h < 4.6:
                    split_damper_1_c_x = int(c_x - (w/4))
                    split_damper_2_c_x = int(c_x + (w/4))

                    split_damper_1_x = x
                    split_damper_2_x = int(x + (w / 2))

                    cv2.rectangle(image, (split_damper_1_x, y), (split_damper_1_x + int(w/2), y + h), (255, 0, 0), 1)
                    cv2.rectangle(image, (split_damper_2_x, y), (split_damper_2_x + int(w / 2), y + h), (255, 0, 0), 1)

                    array_of_damper_locations.append(Damper(int(split_damper_1_c_x), c_y, z, False))
                    array_of_damper_locations.append(Damper(int(split_damper_2_c_x), c_y, z, False))

        while True:
            cv2.imshow('dempers', image)

            key = cv2.waitKey(1)

            # and self.busy is False
            if key == ord('s'):
                break

        if len(array_of_damper_locations) == 0:
            return None, image

        #array_of_damper_locations = self.remove_duplicates(array_of_damper_locations)

        dampers_sorted = self.split_unsorted_array_into_row(array_of_damper_locations)

        return dampers_sorted, image

    def split_unsorted_array_into_row(self, dampers):
        rows = []

        # todo: is this still needed?
        # sorted_by_y = sorted(dampers, key=lambda x: x.y, reverse=True)

        sorted_by_x = sorted(dampers, key=lambda x: x.x, reverse=True)
        smallest_x = sorted_by_x[0].get_x()

        while len(sorted_by_x) != 0:
            current_row = []
            current_row_x = sorted_by_x[0].get_x()
            while True:
                if len(sorted_by_x) > 0 and current_row_x - 30 < sorted_by_x[0].get_x() < current_row_x + 30:
                    current_row.append(sorted_by_x[0])
                    sorted_by_x.remove(sorted_by_x[0])
                    print("this triggers")
                    continue
                break

            # todo: fix this
            #if len(current_row) != 8:
            #    current_row = self.insert_spaces_into_row(current_row, smallest_x)

            current_row = sorted(current_row, key=lambda x:x.y, reverse=True)

            rows.append(current_row)

        return rows

    # Inserts None into spaces where there should be a damper but isn't
    def insert_spaces_into_row(self, row, smallest_x):
        new_row = []

        z = row[0].get_z()

        average_distance_between_centers_in_meters = 0.10
        average_distance_between_centers = self.conversion_service \
            .convert_meters_to_pixels(average_distance_between_centers_in_meters, z)

        i = 0
        counter_distance = 0
        while i < len(row):
            if smallest_x - 10 + (counter_distance * average_distance_between_centers) < row[i].get_x() < smallest_x + \
                    10 + (counter_distance * average_distance_between_centers):
                new_row.append(row[i])

                i += 1
                counter_distance += 1
            else:
                new_row.append(None)
                counter_distance += 1

        nones_to_add = 0
        if len(new_row) < 8:
            nones_to_add = 8 - len(new_row)

        i = 0
        while i < nones_to_add:
            new_row.append(None)

        return new_row

    # todo: change pixel comparisons to meter comparisons and test
    def remove_duplicates(self, unsorted_dampers):
        i = 0

        list_without_duplicates = []

        while i < len(unsorted_dampers):
            j = i + 1

            current_damper = unsorted_dampers[i]
            is_duplicate = False
            while j < len(unsorted_dampers):
                possible_duplicate = unsorted_dampers[j]

                if abs(current_damper.get_x() - possible_duplicate.get_x()) < 10 and abs(current_damper.get_y() - possible_duplicate.get_y()) < 10:
                    is_duplicate = True

                j += 1

            if not is_duplicate:
                list_without_duplicates.append(unsorted_dampers[i])

        return list_without_duplicates
