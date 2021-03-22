import time
import CameraDepalletizer


# Converts centimeters to pixels at a given distance from the camera
def millimeters_to_pixels(distance_to_camera, millimeters):
    return millimeters

# Converts camera coordinates to robot coordinates
def convert_to_robot_coordinates(x, y, z):
    return x, y, z


class RobotState:
    def __init__(self, camera_state, http_service):
        self.camera_state = camera_state
        self.http_service = http_service

        self.busy = False

        self.locations = []

    def main_loop(self):
        while True:
            time.sleep(1)

            if self.busy:
                continue

            dampers = self.camera_state.get_dampers()

            row = 0
            while row < len(dampers):
                column = 0

                this_row = dampers[row]
                while column < len(this_row):

                    self.pick_and_place_row(this_row)

                    # Checks every half second if it can continue
                    while self.busy:
                        time.sleep(0.5)

                row += 1

    def pick_and_place_row(self, row):
        pairs = []
        singles = []

        i = 0
        while i < 8:
            if row[i] and row[i + 1]:
                pairs.append((row[i], row[i + 1]))
            elif row[i]:
                singles.append((row[i], None))
            elif row[i + 1]:
                singles.append((None, row[i + 1]))

        for pair in pairs:

            # Checks every half second if it can continue
            while self.busy:
                time.sleep(0.5)

            damper_1, damper_2 = pair

            if damper_1.get_moved() or damper_1.get_moved():
                continue

            pixel_target_x = (damper_1.get_x() + damper_2.get_x()) / 2
            pixel_target_y = (damper_1.get_y() + damper_2.get_y()) / 2
            target_z = damper_1.get_z()

            robot_target_x, robot_target_y, robot_target_z = convert_to_robot_coordinates(pixel_target_x,
                                                                                               pixel_target_y,
                                                                                               target_z)

            command_string = '[ ]({}.0000,{}.0000,{}.0000)'.format(robot_target_x, robot_target_y, robot_target_z)

            self.http_service.send_command(command_string)
            self.busy = True

            damper_1.set_moved()
            damper_2.set_moved()

        for single in singles:

            # Checks every half second if it can continue
            while self.busy:
                time.sleep(0.5)

                damper_1, damper_2 = single

                if damper_1 is not None:
                    damper = damper_1
                else:
                    damper = damper_2

                # This is to offset the end of arm tool to one side
                pixel_target_x = damper.get_x() - self.centimeters_to_pixels(damper.get_z(), 5)
                pixel_target_y = damper.get_y()
                target_z = damper.get_z()

                robot_target_x, robot_target_y, robot_target_z = convert_to_robot_coordinates(pixel_target_x,
                                                                                                   pixel_target_y,
                                                                                                   target_z)

                command_string = '[ ]({}.0000,{}.0000,{}.0000)'.format(robot_target_x, robot_target_y, robot_target_z)
                self.http_service.send_command(command_string)
                self.busy = True

                damper_1.set_moved()


    def set_robot_done(self):
        self.busy = False

    def set_robot_busy(self):
        self.busy = True
