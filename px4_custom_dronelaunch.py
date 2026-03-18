#!/usr/bin/env python3
"""
Launch file: PX4 SITL + custom drone model + indoor world
Usage:
  ros2 launch <your_pkg> px4_custom_drone.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


# ── Edit these paths to match your system ──────────────────────────────────

PX4_DIR        = os.path.expanduser('~/irobot/px4_autopilot/PX4-Autopilot')
WORLD_FILE     = os.path.join(os.path.dirname(__file__), 'indoor_obstacles.sdf')
MODEL_FILE     = os.path.join(os.path.dirname(__file__), 'model.sdf')
MODEL_NAME     = 'x500_custom'
SPAWN_POSE     = '0,0,0.2,0,0,0'   # x,y,z,roll,pitch,yaw

# ── Environment setup ──────────────────────────────────────────────────────

gz_env = {
    **os.environ,
    # Tell Gazebo where to find your drone mesh files
    'GZ_SIM_RESOURCE_PATH': os.path.join(
        os.path.expanduser('~'),
        'irobot/sjtu_drone_description/models'   # ← adjust to your mesh location
    ),
}

px4_env = {
    **os.environ,
    'PX4_GZ_WORLD':      WORLD_FILE,     # use our custom world
    'PX4_GZ_MODEL_NAME': MODEL_NAME,
    'PX4_SIM_MODEL':     MODEL_NAME,
}


def generate_launch_description():

    # 1 ── Gazebo (headless optional: add --headless-rendering)
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', WORLD_FILE],
        env=gz_env,
        output='screen',
        name='gazebo'
    )

    # 2 ── Spawn custom drone model into running Gazebo world
    #      Delayed 5s to give Gazebo time to start
    spawn_drone = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service',
                    '-s', '/world/indoor_obstacles/create',
                    '--reqtype', 'gz.msgs.EntityFactory',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '5000',
                    '--req',
                    f'sdf_filename: "{MODEL_FILE}", '
                    f'name: "{MODEL_NAME}", '
                    f'pose: {{position: {{x:0, y:0, z:0.2}}}}'
                ],
                output='screen',
                name='spawn_drone'
            )
        ]
    )

    # 3 ── PX4 SITL (connects to already-running Gazebo)
    #      Delayed 8s to let Gazebo + model finish loading
    px4_sitl = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    os.path.join(PX4_DIR, 'build/px4_sitl_default/bin/px4'),
                    os.path.join(PX4_DIR, 'ROMFS/px4fmu_common'),
                    '-s', 'etc/init.d-posix/rcS',
                    '-i', '0',
                    '-d'
                ],
                cwd=os.path.join(PX4_DIR, 'build/px4_sitl_default'),
                env=px4_env,
                output='screen',
                name='px4_sitl'
            )
        ]
    )

    # 4 ── Micro XRCE-DDS Agent (ROS2 ↔ PX4 bridge)
    #      Delayed 12s to let PX4 start first
    dds_agent = TimerAction(
        period=12.0,
        actions=[
            ExecuteProcess(
                cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
                output='screen',
                name='dds_agent'
            )
        ]
    )

    # 5 ── Static TF: map → odom → base_link
    tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # 6 ── Robot State Publisher (publishes TF from SDF joints)
    #      Only needed if you have a URDF alongside the SDF
    # robot_state_pub = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     parameters=[{'robot_description': open(MODEL_FILE).read()}]
    # )

    return LaunchDescription([
        gazebo,
        spawn_drone,
        px4_sitl,
        dds_agent,
        tf_map_odom,
    ])
