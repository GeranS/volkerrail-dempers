from ConversionService import ConversionService

import collections
import cv2
import numpy as np
import pyrealsense2 as rs


class Damper:
    def __init__(self, x, y, z, moved):
        self.x = x
        self.y = y
        self.z = z
        self.moved = moved


class Slat:
    def __init__(self, x, y, z, moved):
        self.x = x
        self.y = y
        self.z = z
        self.moved = moved


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

    def take_picture_with_threshold(self, threshold_z):
        frames = self.pipeline.wait_for_frames()
        self.threshold_filter.set_option(rs.option.max_distance, threshold_z)

        depth_frame = frames.get_depth_frame()
        depth_frame = self.threshold_filter.process(depth_frame)

        depth_colormap = np.asanyarray(self.colorizer.colorize(depth_frame).get_data())

        return depth_colormap

    def get_top_layer_image(self):
        frames = self.pipeline.wait_for_frames()

        # Set distance to 4 meters so it can see the whole pallet
        self.threshold_filter.set_option(rs.option.max_distance, 4)

        depth_frame = frames.get_depth_frame()
        depth_frame = self.threshold_filter.process(depth_frame)

        depth_image = np.asanyarray(depth_frame.get_data())
        depth_image = depth_image[120:360, 120:520]
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

        layer_z = self.conversion_service.get_layer_z(detection_z)

        if layer_z is None:
            return None, None

        return detection_z, layer_z

    # Finds the dampers in the provided white to black image
    def find_dampers(self, detection_z, layer_z):
        image = self.take_picture_with_threshold(layer_z)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        cv2.rectangle(gray, (0, 0), (640, 480), (0, 0, 0), 100)
        print("detection z: " + str(detection_z))
        print("layer z: " + str(layer_z))

        iterations = 8
        kernel = np.ones((5, 5), np.uint8)
        erosion = cv2.erode(gray, kernel, iterations=iterations)
        kernel = np.ones((5, 5), np.uint8)
        dilation = cv2.dilate(erosion, kernel, iterations=iterations)

        edged = cv2.Canny(dilation, 30, 200)

        contours, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cv2.drawContours(image, contours, -1, (0, 0, 255), 2)

        array_of_damper_locations = []

        for contour in contours:
            area = cv2.contourArea(contour)

            m = cv2.moments(contour)

            # More accurate than the bounding box, especially for single damper pickups
            if m['m00'] != 0.0:
                c_x = int(m['m10'] / m['m00'])
                c_y = int(m['m01'] / m['m00'])

                x, y, w, h = cv2.boundingRect(contour)

                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                min_length_damper = 0.06
                max_length_damper = 0.06

                min_width_damper = 0.12
                max_width_damper = 0.17

                h_meters = self.conversion_service.convert_pixels_to_meters(h, detection_z)
                w_meters = self.conversion_service.convert_pixels_to_meters(w, detection_z)

                if h_meters < min_length_damper:
                    print("skipped over blob because length")
                    continue

                if w_meters < min_width_damper:
                    print("skipped over blob because width")
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

                if amount_wide == 1 and max_width_damper < w_meters:
                    print("max: " + str(max_width_damper) + " actual: " + str(w_meters))
                    continue

                if amount_long > 2:
                    print("amount long exceeds 2")
                    continue

                if amount_wide == 99 and amount_long == 2:
                    array_of_damper_locations.append(Damper(c_x, c_y, layer_z, False))
                    cv2.rectangle(image, (x, y), (x + w, y + h), (150, 150, 0), 2)
                else:
                    pxl_h_damper = int(h / minimum_amount_lengthwise)
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
                                cv2.rectangle(image, (bnd_rect_x, bnd_rect_y),
                                              (bnd_rect_x + pxl_w_damper, bnd_rect_y + pxl_h_damper), (255, 0, 0), 2)

                                array_of_damper_locations.append(
                                    Damper(current_damper_x, current_damper_y, layer_z, False))

                            width_counter += 1
                        length_counter += 1

        #cv2.imshow('dempers', image)
        #cv2.waitKey(1)

        if len(array_of_damper_locations) == 0:
            return None

        array_of_damper_locations = self.remove_duplicates(array_of_damper_locations, layer_z)

        dampers_sorted = self.split_unsorted_array_into_columns(array_of_damper_locations)

        return dampers_sorted

    def find_slats(self, layer_z, retry=True):
        image = self.take_picture_with_threshold(layer_z - 0.01)
        image_middle = image[120:360, 120: 520]

        big_kernel = np.ones((8, 8), np.uint8)
        image_middle = cv2.dilate(image_middle, big_kernel, iterations=1)

        image[120:360, 120: 520] = image_middle

        #cv2.imshow('dempers', image)
        #cv2.waitKey(0)

        iterations = 10
        kernel = np.ones((5, 5), np.uint8)
        erosion = cv2.erode(image, kernel, iterations=iterations)
        kernel = np.ones((5, 5), np.uint8)
        dilation = cv2.dilate(erosion, kernel, iterations=iterations)

        #cv2.imshow('dempers', dilation)
        #cv2.waitKey(0)

        edged = cv2.Canny(dilation, 30, 200)

        contours, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        x1, x2, y1, y2 = None, None, None, None

        y_middle = 0

        amount_of_slats_based_on_width = 0
        average_damper_width = self.conversion_service.convert_meters_to_pixels(0.125, layer_z)

        for contour in contours:
            area = cv2.contourArea(contour)

            # todo: change to be dependant on detection_z
            if area < self.conversion_service.scale_pixel_area(10000, 0.63, layer_z):
                continue

            x, y, w, h = cv2.boundingRect(contour)

            amount_of_slats_based_on_width = (w // average_damper_width) + 1
            print("w: " + str(w) + " average_width: " + str(average_damper_width))
            print("detection_z: " + str(layer_z))

            rotated_rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rotated_rect)  # cv2.boxPoints(rect) for OpenCV 3.x
            box = np.int0(box)
            cv2.drawContours(dilation, [box], 0, (255, 255, 255), 30)

            x1 = x
            x2 = x + w
            y1 = y
            y2 = y + h

            y_middle = int((y + y + h) / 2)

        #cv2.imshow('dempers', dilation)
        #cv2.waitKey(0)

        if x1 is None:
            return None

        blank_image = np.zeros(shape=[480, 640, 3], dtype=np.uint8)

        blank_image[y1: y2, x1: x2] = cv2.bitwise_not(dilation[y1: y2, x1: x2])

        inverted = blank_image

        if cv2.countNonZero(cv2.cvtColor(inverted, cv2.COLOR_BGR2GRAY)) < 50:
            return None

        inverted_edge = cv2.Canny(inverted, 30, 200)

        inverted_contours, _ = cv2.findContours(inverted_edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # x and y coordinates of slat edges
        edges = []

        two_centimeters_in_pixels = self.conversion_service.convert_meters_to_pixels(0.02, layer_z)

        for inv_con in inverted_contours:
            cv2.drawContours(inverted, [inv_con], 0, (255, 0, 0), 2)

            x, y, w, h = cv2.boundingRect(inv_con)

            left_edge = (x, y_middle)
            right_edge = (x + w, y_middle)

            if left_edge[0] - two_centimeters_in_pixels > x1:
                edges.append(left_edge)

            if right_edge[0] + two_centimeters_in_pixels < x2:
                edges.append(right_edge)

        edge_columns = []

        for edge in edges:
            if len(edge_columns) == 0:
                edge_columns.append([edge])
            else:
                fits_in_existing_column = False
                for edge_in_column in edge_columns:
                    if abs(edge_in_column[0][0] - edge[0]) < 20:
                        edge_in_column.append(edge)
                        fits_in_existing_column = True
                        break

                if fits_in_existing_column is False:
                    edge_columns.append([edge])

        sorted_edge_columns = sorted(edge_columns, key=lambda c: c[0][0])

        three_centimeters_in_pixels = self.conversion_service.convert_meters_to_pixels(0.03, layer_z)

        slats = []
        counter = 0
        for sorted_column in sorted_edge_columns:
            if counter == 0:
                total_x, total_y = 0, 0

                for point in sorted_column:
                    total_x += point[0]
                    total_y += point[1]

                average_x = int(total_x / len(sorted_column)) - three_centimeters_in_pixels
                average_y = int(total_y / len(sorted_column))

                slats.append(Slat(average_x, average_y, layer_z, False))
            elif counter == len(sorted_edge_columns) - 1:
                total_x, total_y = 0, 0

                for point in sorted_column:
                    total_x += point[0]
                    total_y += point[1]

                average_x = int(total_x / len(sorted_column)) + three_centimeters_in_pixels
                average_y = int(total_y / len(sorted_column))

                slats.append(Slat(average_x, average_y, layer_z, False))
            elif counter % 2 == 1:
                total_x, total_y = 0, 0

                for point in sorted_column:
                    total_x += point[0]
                    total_y += point[1]

                for point in sorted_edge_columns[counter + 1]:
                    total_x += point[0]
                    total_y += point[1]

                average_x = int(total_x / (len(sorted_column) + len(sorted_edge_columns[counter + 1])))
                average_y = int(total_y / (len(sorted_column) + len(sorted_edge_columns[counter + 1])))

                slats.append(Slat(average_x, average_y, layer_z, False))
            counter += 1

        for slat in slats:
            cv2.drawMarker(inverted, (slat.x, slat.y), color=(0, 255, 0), markerType=cv2.MARKER_CROSS, thickness=2)

        #cv2.imshow('dempers', inverted)
        #cv2.waitKey(0)

        if int(counter - 1) != amount_of_slats_based_on_width and retry:
            print("Amount of slats found doesn't match expected amount, trying again.")
            print("Expect: " + str(amount_of_slats_based_on_width) + " Got: " + str(counter - 1))
            print(slats)

            slats = self.find_slats(layer_z, retry=False)

            if len(slats) != int(counter - 1):
                return None

        return slats

    def split_unsorted_array_into_columns(self, dampers):
        columns = []

        sorted_by_x = sorted(dampers, key=lambda x: x.x, reverse=True)

        while len(sorted_by_x) != 0:
            current_column = []
            current_column_x = sorted_by_x[0].x
            while True:
                if len(sorted_by_x) > 0 and current_column_x - 30 < sorted_by_x[0].x < current_column_x + 30:
                    current_column.append(sorted_by_x[0])
                    sorted_by_x.remove(sorted_by_x[0])
                    continue
                break

            current_column = sorted(current_column, key=lambda x: x.y, reverse=True)

            columns.append(current_column)

        columns = self.insert_spaces_into_damper_grid(columns)

        return columns

    # Inserts None into spaces where there should be a damper but isn't
    def insert_spaces_into_damper_grid(self, damper_grid):

        y_list = []

        for row in damper_grid:
            for damper in row:
                y_list.append(damper.y)

        y_rows = []

        counter = 0
        # todo: improvement possible, but check if necessary since I don't want to write any more new for loops than
        #  absolute necessary
        for y_point in y_list:
            match = False
            if counter == 0:
                y_rows.append(y_point)
            else:
                for y_row in y_rows:
                    if abs(y_row - y_point) < 8:
                        match = True
                        break

                if match is False:
                    y_rows.append(y_point)
            counter += 1

        sorted_y_rows = sorted(y_rows)

        new_damper_grid = []

        for column in damper_grid:
            new_column = np.empty(len(sorted_y_rows), dtype=object)

            counter = 0
            for y_row in sorted_y_rows:
                found_match = False
                for damper in column:
                    if abs(y_row - damper.y) < 10:
                        new_column[counter] = damper
                        found_match = True

                if found_match is False:
                    new_column[counter] = None

                counter += 1
            new_damper_grid.append(new_column)

        return new_damper_grid

    def remove_duplicates(self, unsorted_dampers, layer_z):
        unsorted = list(unsorted_dampers.copy())

        four_cm = self.conversion_service.convert_meters_to_pixels(0.04, layer_z)

        i = 0
        while True:
            j = 0
            while j < len(unsorted):
                if i != j:
                    if abs(unsorted[i].x - unsorted[j].x) < four_cm and abs(
                            unsorted[i].y - unsorted[j].y) < four_cm:
                        unsorted.remove(unsorted[j])
                        print("duplicate removed")
                j += 1
            i += 1

            if i == len(unsorted):
                break

        return unsorted
