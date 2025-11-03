from ur3_tcp.srv import GripperCmd

import rclpy
from rclpy.node import Node
import time
import socket

ESP_IP = "192.168.0.120"  # Replace with ESP32's IP
ESP_PORT = 4210


class GripperDriver(Node):
    def __init__(self):
        super().__init__("send_command_to_hardware")
        self.connect_to_esp32()
        self.srv_arduino = self.create_service(GripperCmd, "gripper_cmd", self.send_command_to_esp_callback)
        self.get_logger().info("Gripper driver is ready")

    def connect_to_esp32(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((ESP_IP,ESP_PORT))


    def send_command_to_esp_callback(self, request, response):
        try:
            self.get_logger().info(f"Received command: {request.action}")
            if request.action == "close":
                drv_cmd = "SET_SERVO_POS_SYNC 2750 1350 100 100 10 10"
            if request.action == "open":
                drv_cmd = "SET_SERVO_POS_SYNC 2600 1500 100 100 10 10"
            response_esp = self.send_command_to_esp_and_return_response(drv_cmd)
            self.get_logger().info(f"Response: {response_esp}")
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Error: {str(e)}")
            response_esp = f"Error: {str(e)}"
            response.success = False

        time.sleep(2)
        return response

    def send_command_to_esp_and_return_response(self, command):
        """ Sends a command to the ESP32 over TCP and returns the response. """
        try:
            self.sock.sendall((command + "\n").encode())
            esp_response = self.sock.recv(1024).decode().strip()
            return esp_response
        except Exception as e:
            return f"Connection Error: {str(e)}"
        
    def send_command_with_retries(self, command, max_retries, retry_delay):
        """ Sends a command to the ESP32 and retries if no response is received. """
        attempts = 0
        while attempts < max_retries:
            try:
                self.sock.sendall((command + "\n").encode())
                esp_response = self.sock.recv(1024).decode().strip()
                
                if esp_response:  # If a response is received, return it
                    return esp_response
                
                self.get_logger().warn(f"No response received, retrying ({attempts+1}/{max_retries})...")
            except socket.timeout:
                self.get_logger().warn(f"Timeout occurred, retrying ({attempts+1}/{max_retries})...")
            except Exception as e:
                return f"Connection Error: {str(e)}"
            
            attempts += 1
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=retry_delay))
        
        return "Error: No response received from ESP32 after multiple attempts."


def main():
    rclpy.init()
    node = GripperDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()