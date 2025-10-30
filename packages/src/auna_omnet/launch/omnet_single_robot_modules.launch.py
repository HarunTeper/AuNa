# Copyright 2025 Harun Teper
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


"""Single car omnet module launch file."""
import os
from launch.actions import LogInfo
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Return launch description."""
    communication_type = os.environ.get('COMMUNICATION_TYPE', 'cam')
    if communication_type != 'omnet':
        return LaunchDescription(
            [LogInfo(msg="COMMUNICATION_TYPE is not 'omnet', skipping OMNeT nodes.")])

    # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace')

    # Launch Arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace for ROS nodes and topics'
    )

    # Nodes and other launch files
    omnet_transmitter_cmd = Node(
        package='auna_omnet',
        executable='omnet_transmitter',
        name='omnet_transmitter',
        namespace=namespace,
        arguments={namespace},
        output='screen'
    )

    omnet_receiver_cmd = Node(
        package='auna_omnet',
        executable='omnet_receiver',
        name='omnet_receiver',
        namespace=namespace,
        arguments={namespace},
        output='screen'
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(namespace_arg)

    launch_description.add_action(omnet_transmitter_cmd)
    launch_description.add_action(omnet_receiver_cmd)
    return launch_description
