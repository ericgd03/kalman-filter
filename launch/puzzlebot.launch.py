from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'mini_challenge_5'
    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'puzzlebot.urdf', 
    )
    
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'puzzlebot_view.rviz'
    )

    static_transform_node_0 = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                arguments = ['--x', '0.5', '--y', '0.5', '--z', '0.0',
                                            '--yaw', '0', '--pitch', '0', '--roll', '0.0',
                                            '--frame-id', 'map', '--child-frame-id', 'odom']
                                )

    static_transform_node_1 = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                arguments = ['--x', '0', '--y', '0', '--z', '0.05',
                                            '--yaw', '0', '--pitch', '0', '--roll', '0.0',
                                            '--frame-id', 'base_footprint', '--child-frame-id', 'base_link']
                                )

    static_transform_node_2 = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                arguments = ['--x', '-0.095', '--y', '0', '--z', '-0.04',
                                            '--yaw', '0', '--pitch', '0', '--roll', '0.0',
                                            '--frame-id', 'base_link', '--child-frame-id', 'caster_link']
                                )
    
    static_transform_node_3 = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                arguments = ['--x', '0.053', '--y', '0', '--z', '0.095',
                                            '--yaw', '0', '--pitch', '0', '--roll', '0.0',
                                            '--frame-id', 'base_link', '--child-frame-id', 'base_laser']
                                )

    aruco_3 = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments = ['--x', '0', '--y', '1.5', '--z', '0.0',
                                '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                                '--frame-id', 'map', '--child-frame-id', 'aruco_3']
                )
    
    aruco_4 = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments = ['--x', '-1', '--y', '2', '--z', '0.0',
                                '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                                '--frame-id', 'map', '--child-frame-id', 'aruco_4']
                )

    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([

        # Publicar Transformaciones Estáticas (recomendado)
        static_transform_node_0,
        static_transform_node_1,
        static_transform_node_2,
        static_transform_node_3,
        # aruco_3,
        # aruco_4,
        
        # Publica el URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        # Nodo con dead reckoning y filtro de kalman
        Node(
            package=package_name,
            executable='ekf',
            name='ekf',
            output='screen'
        ),

        # # Aruco detection node
        # Node(
        #     package=package_name,
        #     executable='aruco_detection',
        #     name='aruco_detection',
        #     output='screen'
        # ),

        # # Nodo que simula el comportamiento físico de un robot diferencial. 
        # Node(
        #     package=package_name,
        #     executable='rs_robot',
        #     name='real_sim_robot',
        #     output='screen'
        # ),

        # Nodo controlador proporcional dividido en fases.
        Node(
            package=package_name,
            executable='control',
            name='control_node',
            output='screen'
        ),

        # Nodo para mostrar el recorrido del puzzlebot
        Node(
            package=package_name,
            executable='path_publisher',
            name='path_pub',
            output='screen'
        ),

        # Nodo que publica una secuencia de setpoints Pose2D para que el robot los siga (con retardo)
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package=package_name,
                    executable='set_point',
                    name='set_point_generator',
                    output='screen'
                )
            ]
        ),

        # Visualización en RViz
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_file],
                    output='screen'
                )
            ]
        ),


        # Visualización en rqt_plot con cmd_vel y odom
        # TimerAction(
        #     period=4.0, 
        #     actions=[
        #         Node(
        #             package='rqt_plot',
        #             executable='rqt_plot',
        #             name='rqt_plot',
        #             arguments=[
        #                 '/cmd_vel/linear/x',
        #                 '/odom/pose/pose/position/x',
        #                 '/cmd_vel/angular/z',
        #                 '/odom/pose/pose/position/y'
        #             ],
        #             output='screen'
        #         )
        #     ]
        # )
    ])

