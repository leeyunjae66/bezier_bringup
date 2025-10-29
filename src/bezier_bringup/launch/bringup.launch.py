from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ── 런치 인자 (옵션)
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz2')
    occ_arg  = DeclareLaunchArgument('occ_threshold', default_value='100', description='0~100')

    use_rviz = LaunchConfiguration('rviz')
    occ_th   = LaunchConfiguration('occ_threshold')

    # ── 패키지 내부 맵 경로 (반드시 config/<myname_map.yaml> 존재)
    pkg_share     = FindPackageShare('bezier_bringup')
    packaged_map  = PathJoinSubstitution([pkg_share, 'config', 'tb3_map.yaml'])  # ← 파일명 바꿔 넣기

    # ── map_server (nav2) - lifecycle node
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': packaged_map
        }]
    )

    # ── lifecycle manager to configure→activate map_server
    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    # ── path_planner3/path_generator (console_scripts 등록되어 있어야 함)
    path_gen = Node(
        package='path_planner3',
        executable='path_generator',
        name='path_generator',
        output='screen',
        parameters=[{
            # 네 노드에서 declare_parameter('occ_threshold', 100) 했을 때만 반영됨
            'occ_threshold': occ_th
        }]
    )

    # ── RViz2 (설정 파일 없이 빈 RViz 실행 → GUI에서 네가 직접 설정)
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        rviz_arg, occ_arg,
        map_server,
        lifecycle_mgr,
        path_gen,
        rviz2
    ])
