import collections

import cv2
import numpy as np
import pyrealsense2 as rs

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

    # todo: split z coordinate into a detection z and a damper z from a hardcoded table of distances
    # todo: make detection z more reliable, currently it fails too often by setting the distance too close
    def get_top_layer_image(self):
        frames = self.pipeline.wait_for_frames()

        # Set distance to 4 meters so it can see the whole pallet
        self.threshold_filter.set_option(rs.option.max_distance, 4)

        depth_frame = frames.get_depth_frame()
        depth_frame = self.threshold_filter.process(depth_frame)

        depth_image = np.asanyarray(depth_frame.get_data())
        depth_image = depth_image[80:400, 80:560]
        depth_image = depth_image[depth_image != 0]

        counter = collections.Counter(depth_image)
        # todo: determine most reliable most common set size, the more dampers there are the smaller it can be
        most_common = counter.most_common(500)

        print(most_common)

        smallest_most_common = 10000000

        for m in most_common:
            if m[0] < smallest_most_common:
                smallest_most_common = m[0]

        # todo: Works for now, but requires further tuning for reliability
        closest_object_in_meters = float(smallest_most_common) * self.sensor.get_option(rs.option.depth_units)
        detection_z = closest_object_in_meters + 0.03
        print(detection_z)

        # Set appropriate distance for layer
        self.threshold_filter.set_option(rs.option.max_distance, detection_z)

        frames = self.pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        depth_frame = self.threshold_filter.process(depth_frame)

        depth_colormap = np.asanyarray(self.colorizer.colorize(depth_frame).get_data())

        layer_z = self.conversion_service.get_layer_z(detection_z)

        return depth_colormap, detection_z, layer_z

    # Finds the dampers in the provided white to black image
    def find_dampers(self, image, detection_z, layer_z):
        original_image = image.copy()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        print("detection z: " + str(detection_z))
        print("layer z: " + str(layer_z))

        iterations = 10
        kernel = np.ones((5, 5), np.uint8)
        erosion = cv2.erode(gray, kernel, iterations=iterations)
        kernel = np.ones((5, 5), np.uint8)
        dilation = cv2.dilate(erosion, kernel, iterations=iterations)

        edged = cv2.Canny(dilation, 30, 200)

        while True:
            cv2.imshow('dempers', original_image)
            cv2.imshow('dempers2', dilation)

            key = cv2.waitKey(1)

            # and self.busy is False
            if key == ord('s'):
                break

        contours, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cv2.drawContours(original_image, contours, -1, (0, 0, 255), 2)

        while True:
            cv2.imshow('dempers', original_image)

            key = cv2.waitKey(1)

            # and self.busy is False
            if key == ord('s'):
                break

        array_of_damper_locations = []

        for contour in contours:
            area = cv2.contourArea(contour)

            if area < 3000:
                continue

            m = cv2.moments(contour)

            # More accurate than the bounding box, especially for single damper pickups
            # todo: this whole thing needs to be cleaned up to more readable, perhaps split up into different functions
            if m['m00'] != 0.0:
                c_x = int(m['m10'] / m['m00'])
                c_y = int(m['m01'] / m['m00'])

                x, y, w, h = cv2.boundingRect(contour)

                cv2.rectangle(image, (x, y), (x +w, y +h), (0,255,0), 2)

                min_length_damper = 0.05
                max_length_damper = 0.06

                min_width_damper = 0.12
                max_width_damper = 0.16

                h_meters = self.conversion_service.convert_pixels_to_meters(h, detection_z)
                w_meters = self.conversion_service.convert_pixels_to_meters(w, detection_z)

                minimum_area_damper = self.conversion_service.convert_meters_to_pixels(min_length_damper, detection_z) \
                                      * self.conversion_service.convert_meters_to_pixels(min_width_damper, detection_z)

                if area < minimum_area_damper or h_meters < min_length_damper or w_meters < min_width_damper:
                    continue

                maximum_area_damper = self.conversion_service.convert_meters_to_pixels(max_length_damper, detection_z) \
                                      * self.conversion_service.convert_meters_to_pixels(max_width_damper, detection_z)

                maximum_amount_lengthwise = h_meters // min_length_damper
                minimum_amount_lengthwise = h_meters // max_length_damper

                maximum_amount_widthwise = w_meters // min_width_damper
                minimum_amount_widthwise = w_meters // max_width_damper

                # todo: take into account both min and max
                amount_wide = maximum_amount_widthwise
                amount_long = maximum_amount_lengthwise

                if amount_wide == 1 and amount_long == 1:
                    array_of_damper_locations.append(Damper(c_x, c_y, layer_z, False))
                    cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)
                else:
                    pxl_h_damper = int(h / amount_long)
                    half_height = pxl_h_damper / 2

                    pxl_w_damper = int(w / amount_wide)
                    half_width = pxl_w_damper / 2

                    length_counter = 0
                    while length_counter < amount_long:
                        width_counter = 0
                        while width_counter < amount_wide:
                            current_damper_x = int(x + half_width + (width_counter * pxl_w_damper))
                            current_damper_y = int(y + half_height + (length_counter * pxl_h_damper))

                            crop_to_check_if_damper_exists = dilation.copy()[current_damper_y - 2: current_damper_y + 2,
                                                             current_damper_x - 2: current_damper_x + 2]

                            # Checks if there's actually a damper at the suspected location
                            if cv2.countNonZero(crop_to_check_if_damper_exists) != 0:
                                bnd_rect_x = int(x + (width_counter * pxl_w_damper))
                                bnd_rect_y = int(y + (length_counter * pxl_h_damper))
                                cv2.rectangle(image, (bnd_rect_x, bnd_rect_y), (bnd_rect_x + pxl_w_damper, bnd_rect_y + pxl_h_damper), (255,0,0), 2)

                                array_of_damper_locations.append(Damper(current_damper_x, current_damper_y, layer_z, False))

                            width_counter += 1
                        length_counter += 1

        while True:
            cv2.imshow('dempers', image)

            key = cv2.waitKey(1)

            # and self.busy is False
            if key == ord('s'):
                break

        if len(array_of_damper_locations) == 0:
            return None, image

        # array_of_damper_locations = self.remove_duplicates(array_of_damper_locations)

        dampers_sorted = self.split_unsorted_array_into_row(array_of_damper_locations)

        return dampers_sorted, image

    def find_slats(self, image, detection_z):
        image_middle = image[120:360, 120: 520]

        big_kernel = np.ones((8,8), np.uint8)
        image_middle = cv2.dilate(image_middle, big_kernel, iterations=1)

        image[120:360, 120: 520] = image_middle

        while True:
            cv2.imshow('dempers', image)

            key = cv2.waitKey(1)

            # and self.busy is False
            if key == ord('s'):
                break

        iterations = 10
        kernel = np.ones((5, 5), np.uint8)
        erosion = cv2.erode(image, kernel, iterations=iterations)
        kernel = np.ones((3, 3), np.uint8)
        dilation = cv2.dilate(erosion, kernel, iterations=iterations)

        while True:
            cv2.imshow('dempers', dilation)

            key = cv2.waitKey(1)

            # and self.busy is False
            if key == ord('s'):
                break

        edged = cv2.Canny(dilation, 30, 200)

        contours, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        x1, x2, y1, y2 = None, None, None, None

        for contour in contours:
            area = cv2.contourArea(contour)

            #todo: change to be dependant on detection_z
            if area < 10000:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            rotated_rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rotated_rect)  # cv2.boxPoints(rect) for OpenCV 3.x
            box = np.int0(box)
            cv2.drawContours(dilation, [box], 0, (255, 255, 255), 25)

            x1 = x
            x2 = x + w
            y1 = y
            y2 = y + h

        while True:
            cv2.imshow('dempers', dilation)

            key = cv2.waitKey(1)

            # and self.busy is False
            if key == ord('s'):
                break

        blank_image = np.zeros(shape=[480, 640, 3], dtype=np.uint8)

        blank_image[y1: y2, x1: x2] = cv2.bitwise_not(dilation[y1: y2, x1: x2])

        inverted = blank_image

        while True:
            cv2.imshow('dempers', inverted)

            key = cv2.waitKey(1)

            # and self.busy is False
            if key == ord('s'):
                break

        # todo: anything past this line may or may not no longer be relevant

        #edged = cv2.Canny(inverted_cropped, 30, 200)

        contours, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        possible_slats = [[(0, 0)]]
        slats = []

        two_centimeters_in_pixels = self.conversion_service.convert_meters_to_pixels(0.02, detection_z)

        # Finding the empty spaces between the dampers and slats
        for contour in contours:
            x, y, w, _ = cv2.boundingRect(contour)

            y = y - two_centimeters_in_pixels
            x = x + int(w/2)

            for possible in possible_slats:
                if abs(y - possible[0][1]) > 5:
                    possible_slats.append([(x, y)])
                else:
                    possible.append((x, y))

        for possible_slat in possible_slats:
            sorted_possible_slat = sorted(possible_slat, key=lambda ps: ps[0])
            leftmost_coord = sorted_possible_slat[0]
            rightmost_coord = sorted_possible_slat[len(sorted_possible_slat) - 1]

            average_x = (leftmost_coord[0] + rightmost_coord[0]) / 2
            average_y = (leftmost_coord[1] + rightmost_coord[1]) / 2

            slats.append((average_x, average_y))

        return slats

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
            # if len(current_row) != 8:
            #    current_row = self.insert_spaces_into_row(current_row, smallest_x)

            current_row = sorted(current_row, key=lambda x: x.y, reverse=True)

            rows.append(current_row)

        return rows

    # Inserts None into spaces where there should be a damper but isn't
    # todo: fix this
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

                if abs(current_damper.get_x() - possible_duplicate.get_x()) < 10 and abs(
                        current_damper.get_y() - possible_duplicate.get_y()) < 10:
                    is_duplicate = True

                j += 1

            if not is_duplicate:
                list_without_duplicates.append(unsorted_dampers[i])

        return list_without_duplicates
