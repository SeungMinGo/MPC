#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
import launch.logging

def generate_launch_description():
# 런치 설명 객체 생성
  ld = LaunchDescription()

    # Names and poses of the robots
  robots = [
      {'name': 'robot1', 'x_pose': '0', 'y_pose': '3.0', 'z_pose': 0.01},
      # {'name': 'tb2', 'x_pose': '0', 'y_pose': '1.0', 'z_pose': 0.01},
      # {'name': 'tb3', 'x_pose': '0', 'y_pose': '-1.0', 'z_pose': 0.01},
      # {'name': 'tb4', 'x_pose': '0', 'y_pose': '2.0', 'z_pose': 0.01},
      # {'name': 'tb5', 'x_pose': '0', 'y_pose': '-2.0', 'z_pose': 0.01},
        # ...
        # ...
      ]

# 사용할 터틀봇 모델 지정
  TURTLEBOT3_MODEL = 'burger'
# 시뮬레이션 시간 사용 여부 결정
  use_sim_time = LaunchConfiguration('use_sim_time', default='true')
# 'use_sim_time' 변수 선언
  declare_use_sim_time = DeclareLaunchArgument(
    name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
  )

# 패키지의 공유 디렉토리 경로는 해당 패키지 내에서 사용되는 리소스 및 노드 파일을 실행하는데 필요한
# 경로를 제공한다. 특히 런치 파일에서 다른 노드 파일을 실행하려면 해당 노드 파일이 포함된 패키지의 디렉토리 경로를 알아야됨.
# 'turtlebot3_multi_robot' 패키지의 공유 디렉토리 경로
  turtlebot3_multi_robot = get_package_share_directory('turtlebot3_gazebo')

# 로봇의 URDF 파일 경로
  urdf = os.path.join(
    turtlebot3_multi_robot, 'urdf', 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
  )

  """
    # 사용할 월드 파일 경로
  world = os.path.join(
      get_package_share_directory('turtlebot3_multi_robot'),
      'worlds', 'multi_robot_world.world')
    """
# Gazebo 서버 런치 포함
  gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        # 사용할 월드 파일 경로
        #launch_arguments={'world': world}.items(),
    )
# Gazebo 클라이언트 런치 포함
  gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ),
  )

    # 선언한 변수들을 ld에 추가
  ld.add_action(declare_use_sim_time)
  ld.add_action(gzserver_cmd)
  ld.add_action(gzclient_cmd)



# TF 관련 토픽 리매핑
# 맵 서버 및 라이프사이클 관리자 노드 생성 및 추가 (필수적인지는 알아봐야됨)
  remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

  last_action = None

# 로봇 인스턴스를 Gazebo에 스폰
  for robot in robots:

    namespace = [ '/' + robot['name'] ]

# 각 로봇 인스턴스에 대한 상태 발행 노드 생성
    # turtlebot_state_publisher = Node(
    #       package='robot_state_publisher',
    #         namespace=namespace,
    #         # 무슨 파일? -> foxy에 기본적으로 있는 파일
    #         executable='robot_state_publisher',
    #         output='screen',
    #         parameters=[{'use_sim_time': use_sim_time,
    #                         'publish_frequency': 10.0}],
    #         remappings=remappings,
    #         arguments=[urdf],
    #     )

# 터틀봇 인스턴스를 Gazebo에 스폰하는 노드 생성
    spawn_turtlebot3_burger = Node(
            package='gazebo_ros',
            # 무슨 파일? -> gazebo에 기본적으로 있는 파일
            executable='spawn_entity.py',
            arguments=[
                '-file', os.path.join(turtlebot3_multi_robot,'models', 'turtlebot3_' + TURTLEBOT3_MODEL, 'model.sdf'),
                '-entity', robot['name'],
                '-robot_namespace', namespace,
                '-x', robot['x_pose'], '-y', robot['y_pose'],
                '-z', '0.01', '-Y', '0.0',
                '-unpause',
            ],
            output='screen',
        )

    # 첫번째 로봇이면 노드를 직접 추가하고, 그렇지 않으면 이전 로봇의 생성 완료를 기다림
    if last_action is None:
            # Call add_action directly for the first robot to facilitate chain instantiation via RegisterEventHandler
            # ld.add_action(turtlebot_state_publisher)
            ld.add_action(spawn_turtlebot3_burger)

    else:
            # Use RegisterEventHandler to ensure next robot creation happens only after the previous one is completed.
            # Simply calling ld.add_action for spawn_entity introduces issues due to parallel run.
            spawn_turtlebot3_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[spawn_turtlebot3_burger
                            # turtlebot_state_publisher
                            ],
                )
            )

            ld.add_action(spawn_turtlebot3_event)

        # Save last instance for next RegisterEventHandler
    last_action = spawn_turtlebot3_burger

  for robot in robots:

      namespace = [ '/' + robot['name'] ]

# 초기 위치를 설정하는 노드 생성
        # Create a initial pose topic publish call
      message = '{header: {frame_id: map}, pose: {pose: {position: {x: ' + \
          robot['x_pose'] + ', y: ' + robot['y_pose'] + \
          ', z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0000000}}, }}'

      initial_pose_cmd = ExecuteProcess(
          cmd=['ros2', 'topic', 'pub', '-1', '--qos-reliability', 'reliable', namespace + ['/initialpose'],
              'geometry_msgs/PoseWithCovarianceStamped', message],
          output='screen'
      )


      post_spawn_event = RegisterEventHandler(
          event_handler=OnProcessExit(
              target_action=last_action,
              on_exit=[initial_pose_cmd],
          )
      )
      last_action = initial_pose_cmd
      ld.add_action(post_spawn_event)

  return ld

ld = LaunchDescription()

