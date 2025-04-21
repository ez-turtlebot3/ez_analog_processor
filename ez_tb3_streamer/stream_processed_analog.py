import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import json
import socket
import threading
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class ProcessedAnalogStreamer(Node):
    def __init__(self):
        super().__init__('processed_analog_streamer')
        
        # Load configuration from YAML file
        self.port = self.load_config()
        
        # Initialize socket and client list
        self._tcp_clients = []
        
        # Subscribe to the /processed_analog topic
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/processed_analog',
            self.topic_callback,
            10)
            
        self.get_logger().info(f'Starting to stream /processed_analog topic on port {self.port}')
        
        # Start TCP server in a separate thread
        self.server_thread = threading.Thread(target=self.run_server)
        self.server_thread.daemon = True
        self.server_thread.start()

    def load_config(self):
        """Load configuration from YAML file"""
        try:
            # Look for configuration in the standard ROS 2 package location
            pkg_dir = get_package_share_directory('ez_tb3_streamer')
            config_path = os.path.join(pkg_dir, 'config', 'sensors.yaml')
            
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
                
            # Try to get port from sensors.yaml, default to 5003 if not found
            port = config.get('streaming', {}).get('processed_analog', {}).get('port', 5003)
            self.get_logger().info(f'Loaded port {port} from config file')
            return port
            
        except Exception as e:
            self.get_logger().error(f'Error loading config: {e}. Using default port 5003')
            return 5003

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
                    self._tcp_clients.append(client_socket)
                except Exception as e:
                    self.get_logger().error(f'Error accepting connection: {e}')
        except Exception as e:
            self.get_logger().error(f'Server error: {e}')

    def topic_callback(self, msg):
        """Process incoming /processed_analog messages and forward to clients"""
        if not self._tcp_clients:
            return
            
        try:
            # Extract the data array
            processed_data = list(msg.data)
            
            # Create a simple JSON structure
            data = {
                'data': processed_data
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
        for client in self._tcp_clients[:]:
            try:
                client.send(data_bytes)
            except Exception as e:
                self.get_logger().error(f'Error sending to client: {e}')
                try:
                    client.close()
                except:
                    pass
                try:
                    self._tcp_clients.remove(client)
                except:
                    pass

def main():
    rclpy.init()
    
    try:
        node = ProcessedAnalogStreamer()
        print(f"Streaming /processed_analog on port {node.port}")
        print("Press Ctrl+C to terminate")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()