#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.actions import GroupAction

def generate_launch_description():
    return LaunchDescription([
        # 0초: PX4 SITL + 브리지 노드 동시에 실행
        GroupAction([
            ExecuteProcess(
                cmd=['make', 'px4_sitl', 'gz_x500_depth'],
                cwd='/home/iasl/drone_project/PX4-Autopilot',
                output='screen'
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='depth_camera_bridge',
                arguments=[
                    '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image'
                ],
                output='screen'
            )
        ]),

        # 10초 후 QGroundControl 실행
        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                    cmd=['/home/iasl/Downloads/QGroundControl.AppImage'],
                    output='screen'
                )
            ]
        ),

        # 20초 후 MicroXRCEAgent 실행
        TimerAction(
            period=20.0,
            actions=[
                ExecuteProcess(
                    cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
                    output='screen'
                )
            ]
        ),

        # 25초 후 landing_pad 모델 삽입
        TimerAction(
            period=25.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'gz', 'service', '-s', '/world/default/create',
                        '--reqtype', 'gz.msgs.EntityFactory',
                        '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '1000',
                        '--req',
                        'sdf_filename: "file:///home/iasl/.gz/fuel/models/Apriltag36_11_00001_small/model.sdf"\n'
                        'name: "landing_pad"\n'
                        'pose: {position: {x: 3.0, y: 0.0, z: 0.0}}'
                    ],
                    output='screen'
                )
            ]
        ),

        # 30초 후 test 노드 실행
        TimerAction(
            period=30.0,
            actions=[
                Node(
                    package='rl_drone',
                    executable='test',
                    output='screen'
                )
            ]
        )
    ])
