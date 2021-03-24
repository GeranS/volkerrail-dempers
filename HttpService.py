import socket
import _thread

robot_client_ip = "192.168.0.1"
robot_client_port = 22


class HttpService:
    def __init__(self, robot_state):
        self.robot_state = robot_state
        server_socket = socket.socket()

        host = socket.gethostname()
        port = 8000

        server_socket.bind((host, port))
        server_socket.listen(2)

        print("Waiting for robot to connect")
        #conn, _ = server_socket.accept()
        #self.client = conn

        #_thread.start_new_thread(self.listen_for_response, ())

    def send_command(self, command_string):
        # todo:
        # everything
        print('http message: ' + command_string)
        return command_string

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
