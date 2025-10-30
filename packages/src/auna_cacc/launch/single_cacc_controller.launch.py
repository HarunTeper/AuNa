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


import os
from launch_ros.actions import Node, PushRosNamespace
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_context import LaunchContext


def include_launch_description(context: LaunchContext):
    """Return launch description (using environment variables only)."""
    robot_index = int(os.environ.get('ROBOT_INDEX', '1'))

    # Create robot namespace based on index
    if int(robot_index) >= 0:
        namespace = f'robot{robot_index}'
    else:
        namespace = ''

    # auna_common paths
    auna_common_path = "/home/ubuntu/workspace/auna_common"
    cacc_config_file = os.path.join(
        auna_common_path,
        'config',
        'cacc',
        'cacc_controller.yaml'
    )

    launch_description_content = []
    launch_description_content.append(PushRosNamespace(namespace))

    # CACC Controller Node
    cacc_controller_node = Node(
        package='auna_cacc',
        executable='cacc_controller',
        name='cacc_controller',
        output='screen',
        parameters=[cacc_config_file],
    )

    launch_description_content.append(cacc_controller_node)

    return launch_description_content


def generate_launch_description():
    """Return launch description."""
    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
