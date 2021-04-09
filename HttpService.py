import socket
import time
import struct
import _thread

robot_client_ip = "192.168.0.3"
robot_client_port = 22

plc_client_ip = "192.168.0.1"
plc_client_port = 2000


# todo: Allow for reconnections and for the robot to be started first
# todo: Add functionality for communicating with PLC
class HttpService:
    def __init__(self, robot_state):
        self.robot_state = robot_state
        self.server_socket = socket.socket()

        host = socket.gethostname()
        port = 8000

        self.server_socket.bind((host, port))
        self.server_socket.listen(2)

        self.robot_client = None
        self.plc_client = None

        self.wait_for_robot_connection()
        # self.connect_to_plc()

    def wait_for_robot_connection(self):
        print("Waiting for robot to connect...")
        conn, _ = self.server_socket.accept()
        self.robot_client = conn
        _thread.start_new_thread(self.listen_for_robot_messages, ())

    def connect_to_plc(self):
        while True:
            print("Connecting to PLC...")
            try:
                self.server_socket.connect((plc_client_ip, plc_client_port))
                break
            except:
                print("Failed to connect to PLC. Trying again in one second.")
                time.sleep(1)

    def send_code_to_plc(self, code):
        send_data = struct.pack(">h", code)
        print("message to PLC: " + str(code))
        #self.plc_client.send(send_data)
        send_data = struct.pack(">h", 99)
        #self.plc_client.send(send_data)

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

    def send_command(self, message_string):
        print('message to robot: ' + message_string)
        self.robot_client.send(message_string.encode())
        return message_string

    def listen_for_plc_messages(self):
        while True:
            try:
                message = self.plc_client.recv(1024)
            except ConnectionResetError as e:
                print("Error while receiving data from PLC. Error message: ", str(e))
                break

            if not message:
                break

            if len(message) == 2:
                code = int(struct.unpack(">h", message[0:2])[0])

                if code == 0:
                    # Start auto mode
                    if self.robot_state.auto is False:
                        self.robot_state.start_automatic_mode()
                elif code == 1:
                    print("pause and go to safe")

            else:
                # Show error message when the number of bytes is not 2
                print("Received too little or too much bytes. 2 needed, received: ", len(message))

        self.plc_client.close()

    # todo: second listener for communication with PLC
    def listen_for_robot_messages(self):
        while True:
            try:
                message = self.robot_client.recv(1024).decode()
            except ConnectionResetError as e:
                print("Error while receiving data from robot. Error message: ", str(e))
                break

            if not message:
                break

            print('from robot:' + str(message))

            if str(message) == 'DONE':
                self.robot_state.set_robot_done()
            elif str(message) == 'PLACED':
                self.send_code_to_plc(2)

        self.robot_client.close()
