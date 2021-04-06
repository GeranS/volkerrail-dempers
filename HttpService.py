import socket
import time
import _thread

robot_client_ip = "192.168.0.3"
robot_client_port = 22

# todo: Allow for reconnections and for the robot to be started first
# todo: Add functionality for communicating with PLC
class HttpService:
    def __init__(self, robot_state):
        self.robot_state = robot_state
        server_socket = socket.socket()

        host = socket.gethostname()
        port = 8000

        server_socket.bind((host, port))
        server_socket.listen(2)

        print("Waiting for robot to connect...")
        conn, _ = server_socket.accept()
        self.client = conn

        _thread.start_new_thread(self.listen_for_response, ())

    def send_move_command(self, robot_x, robot_y, robot_z):
        message_string = 'MOVE'
        self.send_command(message_string)

        # necessary to not overwhelm robot with messages, can be lower
        time.sleep(0.5)

        # todo: temporary fix, has to be changed back once new z system is implemented
        message_string = '({x:.4f},{y:.4f},{z:.4f})'.format(x=robot_x, y=robot_y, z=0.063)
        self.send_command(message_string)

    def send_safe_command(self):
        message_string = 'SAFE'
        self.send_command(message_string)

    def send_command(self, message_string):
        # self.client.send(message_string)
        print('http message: ' + message_string)
        self.client.send(message_string.encode())
        return message_string

    # todo: second listener for communication with PLC
    def listen_for_response(self):
        while True:
            try:
                message = self.client.recv(1024).decode()
            except ConnectionResetError:
                print(ConnectionResetError)
                break

            if str(message) == 'DONE':
                print('from robot: DONE')
                self.robot_state.set_robot_done()
