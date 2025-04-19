import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
import json
import socket
import threading
import argparse

class AnalogPinsStreamer(Node):
    def __init__(self, port=5003):
        super().__init__('analog_pins_streamer')
        
        # Initialize socket and client list - use _tcp_clients instead of clients
        self._tcp_clients = []  # Changed from self.clients to self._tcp_clients
        self.port = port
        
        # Subscribe specifically to the /analog_pins topic with known type
        self.subscription = self.create_subscription(
            UInt16MultiArray,
            '/analog_pins',
            self.topic_callback,
            10)
            
        self.get_logger().info(f'Starting to stream /analog_pins topic on port {port}')
        
        # Start TCP server in a separate thread
        self.server_thread = threading.Thread(target=self.run_server)
        self.server_thread.daemon = True
        self.server_thread.start()

    def run_server(self):
        """Run the TCP server that clients connect to"""
        try:
            # Create TCP socket
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind(('0.0.0.0', self.port))
            server_socket.listen(5)
            self.get_logger().info(f'TCP server listening on port {self.port}')
            
            while True:
                try:
                    client_socket, addr = server_socket.accept()
                    self.get_logger().info(f'Client connected from {addr}')
                    self._tcp_clients.append(client_socket)  # Changed from self.clients
                except Exception as e:
                    self.get_logger().error(f'Error accepting connection: {e}')
        except Exception as e:
            self.get_logger().error(f'Server error: {e}')

    def topic_callback(self, msg):
        """Process incoming /analog_pins messages and forward to clients"""
        if not self._tcp_clients:  # Changed from self.clients
            return
            
        try:
            # Extract just the data array - this is what we care about
            analog_data = list(msg.data)
            
            # Create a simple JSON structure
            data = {
                'data': analog_data
            }
            
            # Convert to JSON string with newline delimiter
            json_data = json.dumps(data) + '\n'
            
            # Send to all connected clients
            self.send_to_clients(json_data)
            
        except Exception as e:
            self.get_logger().error(f'Error processing message: {e}')
    
    def send_to_clients(self, data_str):
        """Send data to all connected clients"""
        data_bytes = data_str.encode('utf-8')
        
        # Copy the list to avoid modification during iteration
        for client in self._tcp_clients[:]:  # Changed from self.clients
            try:
                client.send(data_bytes)
            except Exception as e:
                self.get_logger().error(f'Error sending to client: {e}')
                try:
                    client.close()
                except:
                    pass
                try:
                    self._tcp_clients.remove(client)  # Changed from self.clients
                except:
                    pass

def main():
    parser = argparse.ArgumentParser(description='Stream /analog_pins topic to TCP clients')
    parser.add_argument('--port', type=int, default=5003,
                       help='TCP port to use for streaming (default: 5003)')
    
    rclpy.init()
    
    args = parser.parse_args()
    
    try:
        node = AnalogPinsStreamer(args.port)
        print(f"Streaming /analog_pins on port {args.port}")
        print("Press Ctrl+C to terminate")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()