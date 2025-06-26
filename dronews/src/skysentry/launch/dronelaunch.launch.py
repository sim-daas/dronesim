from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Create the LaunchDescription object
    ld = LaunchDescription()

    # Define the node to publish keyboard presses
    keyboard_publisher_node = Node(
        package='skysentry',
        executable='roskeypub.py',
        name='keyboard_publisher_node',
        output='screen'
    )

    # Define the node to send MAVLink commands
    mavlink_ros_node = Node(
        package='skysentry',
        executable='mavlinkros.py',
        name='mavlink_ros_node',
        output='screen'
    )

    # Define the node to bridge camera images from Gazebo to ROS 2
    gz_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_camera_bridge',
        arguments=[
            # Bridge Gazebo camera topic to ROS
            # The Gazebo topic is on the left, the ROS message type in the middle
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        remappings=[
            # Remap the default bridged topic '/camera' to '/camera/image_raw'
            ('/camera', '/camera/image_raw')
        ],
        output='screen'
    )

    # Add the nodes to the LaunchDescription
    ld.add_action(keyboard_publisher_node)
    ld.add_action(mavlink_ros_node)
    ld.add_action(gz_camera_bridge)

    return ld
