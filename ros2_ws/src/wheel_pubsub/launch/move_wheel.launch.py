from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    g29_ff = Node(
            package="ros_g29_force_feedback",
            executable="g29_force_feedback",
            name="g29_force_feedback",
            parameters=[
                {'device_name': "/dev/input/event0"},
                {"gain": 40}
            ],
            output="screen"
            )

    controller_node = Node(
        package="wheel_pubsub",
        executable="wheel_buffer",
        output='both'
    )

    client_node = Node(
        package="wheel_pubsub",
        executable="listener",
        output='both'
    )

    # ld.add_action(joy)
    ld.add_action(controller_node)
    ld.add_action(client_node)
    ld.add_action(g29_ff)

    return ld
