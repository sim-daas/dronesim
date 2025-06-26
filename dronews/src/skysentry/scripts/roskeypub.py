import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(String, 'keyboard_input', 10)
        self.timer = self.create_timer(0.1, self.publish_keypresses)  # 10Hz update rate

        # Initialize pygame for capturing keyboard input
        pygame.init()
        self.window = pygame.display.set_mode((200, 200))  # Dummy window to capture input

        self.get_logger().info("Keyboard Publisher Node Initialized.")

    def publish_keypresses(self):
        """
        Captures keyboard input and publishes active keys as a comma-separated string.
        """
        keys = []
        pygame.event.pump()  # Process event queue
        key_input = pygame.key.get_pressed()

        # Define the key mappings (change/add keys as needed)
        key_map = {
            "LEFT": pygame.K_LEFT,
            "RIGHT": pygame.K_RIGHT,
            "UP": pygame.K_UP,
            "DOWN": pygame.K_DOWN,
            "w": pygame.K_w,
            "s": pygame.K_s,
            "a": pygame.K_a,
            "d": pygame.K_d,
            "i": pygame.K_i,
            "r": pygame.K_r,
            "l": pygame.K_l,
        }

        # Check which keys are pressed
        for key_name, key_code in key_map.items():
            if key_input[key_code]:
                keys.append(key_name)

        # Convert list to comma-separated string and publish.
        # This will send an empty string if no keys are pressed, telling
        # the subscriber to stop movement.
        msg = String()
        msg.data = ",".join(keys)
        self.publisher_.publish(msg)

        # Only log when keys are actually being pressed to avoid spam.
        if keys:
            self.get_logger().info(f"Published: '{msg.data}' (type: {type(msg.data)})")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Publisher Node Stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
