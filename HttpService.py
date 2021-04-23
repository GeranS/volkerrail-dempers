import socket
import sys
import time
import struct
import _thread

robot_client_ip = "192.168.0.4"
robot_client_port = 22

plc_client_ip = "192.168.0.1"
plc_client_port = 2000


# todo: Allow for reconnections and for the robot to be started first
class HttpService:
    def __init__(self, robot_state):
        self.robot_state = robot_state

        host = socket.gethostname()
        port = 8000

        # Socket for robot communication
        self.robot_socket = socket.socket()
        self.robot_socket.bind((host, port))
        self.robot_socket.listen(2)

        # Socket for PLC communication
        self.socket_plc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.robot_client = None

        self.wait_for_robot_connection()
        self.connect_to_plc()

    def wait_for_robot_connection(self):
        print("Waiting for robot to connect...")
        conn, _ = self.robot_socket.accept()
        self.robot_client = conn
        _thread.start_new_thread(self.listen_for_robot_messages, ())

    def connect_to_plc(self):
        while True:
            print("Connecting to PLC...")
            try:
                self.socket_plc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket_plc.connect((plc_client_ip, plc_client_port))
                _thread.start_new_thread(self.listen_for_plc_messages, ())
                break
            except Exception as e:
                print("Failed to connect to PLC. Trying again in one second.")
                print(e)
                time.sleep(1)

    def send_code_to_plc(self, code):
        if code == 3:
            _thread.start_new_thread(self.send_delayed_code_to_plc, ())
            return

        send_data = struct.pack(">h", code)
        print("message to PLC: " + str(code))
        self.socket_plc.send(send_data)
        send_data = struct.pack(">h", 99)
        self.socket_plc.send(send_data)

    def send_delayed_code_to_plc(self):
        time.sleep(7)
        send_data = struct.pack(">h", 3)
        print("message to PLC: " + str(3))
        self.socket_plc.send(send_data)
        send_data = struct.pack(">h", 99)
        self.socket_plc.send(send_data)


    def send_move_command(self, robot_x, robot_y, robot_z, config):
        # for config, 0 means both sides, 1 means left, 2 means right
        message_string = 'MOVE'
        self.send_command(message_string)

        # necessary to not overwhelm robot with messages, can be lower
        time.sleep(0.2)

        message_string = '({x:.4f},{y:.4f},{z:.4f},{config})'.format(x=robot_x, y=robot_y, z=robot_z, config=config)
        self.send_command(message_string)

    def send_picture_command(self):
        message_string = "PICTURE"
        self.send_command(message_string)

    def send_move_slats_command(self, robot_x, robot_y, robot_z):
        message_string = 'SLAT'
        self.send_command(message_string)

        # necessary to not overwhelm robot with messages, can be lower
        time.sleep(0.2)

        message_string = '({x:.4f},{y:.4f},{z:.4f})'.format(x=robot_x, y=robot_y, z=robot_z)
        self.send_command(message_string)

    def send_safe_command(self):
        message_string = 'SAFE'
        self.send_command(message_string)

    def send_stop_command(self):
        message_string = 'STOP'
        self.send_command(message_string)

    def send_shutdown_command(self):
        message_string = 'END'
        self.send_command(message_string)

    def send_command(self, message_string):
        print('message to robot: ' + message_string)
        self.robot_client.send(message_string.encode())
        return message_string

    def listen_for_plc_messages(self):
        while True:
            try:
                message = self.socket_plc.recv(1024)
            except ConnectionResetError as e:
                print("Error while receiving data from PLC. Error message: ", str(e))
                break
            except Exception as e:
                print(e)
                break

            if not message:
                print("Message is none.")
                break

            if len(message) == 2:
                code = int(struct.unpack(">h", message[0:2])[0])

                print("message from plc: " + str(code))

                if code == 0:  # Start auto
                    print('Message from PLC: Start automatic mode.')
                    if self.robot_state.auto is False:
                        self.robot_state.start_automatic_mode()

                elif code == 1:  # Move to safe
                    print('Message from PLC: Move to safe.')
                    self.robot_state.paused = True
                    self.send_safe_command()

                elif code == 2:  # Continue
                    print('Message from PLC: Continue.')
                    self.robot_state.paused = False

                elif code == 3:  # Place damper
                    print('Message from PLC: Place Damper.')
                    self.robot_state.place_next = True

                elif code == 4:  # Emergency safe
                    print('Message from PLC: Emergency Stop.')
                    self.send_stop_command()

                elif code == 5:  # Shut down
                    print('Message from PLC: Shutdown.')
                    self.robot_state.shutdown()
                    sys.exit()

            else:
                # Show error message when the number of bytes is not 2
                print("Received too little or too much bytes. 2 needed, received: ", len(message))

        print("Closing PLC socket")
        self.socket_plc.close()

        self.connect_to_plc()

    def listen_for_robot_messages(self):
        while True:
            try:
                message = self.robot_client.recv(1024).decode()
            except ConnectionResetError as e:
                print("Error while receiving data from robot. Error message: ", str(e))
                break

            if not message:
                break

            print('message from robot:' + str(message))

            if str(message) == 'DONE':
                self.robot_state.set_robot_done()
            elif str(message) == 'PLACED':
                self.send_code_to_plc(2)  # Damper placed

        self.robot_client.close()
