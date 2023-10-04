import launch
from launch import LaunchDescription
from launch_ros.actions import Node

# パッケージへのPATHを知るために必要
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    return LaunchDescription([
        
        launch.actions.LogInfo(
            msg="カメラを起動するnode."
        ),
        
        # launch.actions.TimerAction(period=3.0, actions=[
        #     launch.actions.LogInfo(
        #         msg="It's been three seconds since the launch"
        #     )
        # ]),

        # Node for v4l2_camera
        Node(
            package         = 'v4l2_camera',         # package name
            namespace       = 'v4l2_camera',         # namespace for launching node
            executable      = 'v4l2_camera_node',    # file name to run
            name            = 'v4l2_camera',         # node name
            output          = 'screen',            # option to display standard output on console
            parameters      = []
        ),

        # Node for Rviz2
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('human_action_map'), 'camera.rviz')]
        ),

        # # Node for turtlesim
        # Node(
        #     package         = 'turtlesim',         # package name
        #     namespace       = 'turtlesim',         # namespace for launching node
        #     executable      = 'turtlesim_node',    # file name to run
        #     name            = 'turtlesim',         # node name
        #     output          = 'screen',            # option to display standard output on console
        #     parameters      = [ {'background_r': 255},
        #                         {'background_g': 255},
        #                         {'background_b': 0}
        #                         ]),
        
        # # Node for teleop_key to control turtles
        # Node(
        #     package         = 'turtlesim',         # package name
        #     namespace       = 'turtlesim',         # namespace for launching node
        #     executable      = 'turtle_teleop_key', # file name to run
        #     name            = 'teleop_turtle',     # node name
        #     prefix          = 'xterm -e',          # option: run this another terminal here; xterm requires you to install
        #                         ),
        
    ])