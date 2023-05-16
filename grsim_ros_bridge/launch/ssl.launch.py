from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    grsim_node = Node(
        package='grsim_ros_bridge',
        executable='run_grsim',
        name='run_grsim'
    )
    vision_node =  Node(
        package='vision_comm',
        executable='vision_comm',
        name='vision_comm',
        parameters=['1']
    )
    grsim_bridge_node = Node(
        package='grsim_ros_bridge',
        executable='grsim_ros_bridge',
        name='grsim_ros_bridge'
    )
    test_ssl_node = Node(
        package='test_ssl',
        executable='test_ssl',
        name='test_ssl'
    )

    ld.add_action(grsim_node)
    ld.add_action(vision_node)
    ld.add_action(grsim_bridge_node)
    ld.add_action(test_ssl_node)
    return ld