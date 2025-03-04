# import os
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     # Define launch arguments
#     model_arg = DeclareLaunchArgument('model', default_value='a0509', description='Robot Model')
#     color_arg = DeclareLaunchArgument('color', default_value='blue', description='Robot Color')
#     name_arg  = DeclareLaunchArgument('name',  default_value='dsr01', description='Robot Namespace')

#     # Get package share directories
#     dsr_description2_share = get_package_share_directory('dsr_description2')
#     dsr_controller2_share  = get_package_share_directory('dsr_controller2')
#     dsr_bringup2_share = get_package_share_directory('dsr_bringup2')

#     # Include the description launch file
#     description_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(dsr_description2_share, 'launch', 'dsr_description.launch.py')
#         ),
#         launch_arguments={'model': LaunchConfiguration('model'),
#                           'color': LaunchConfiguration('color')}.items()
#     )

#     # Include the controller launch file
#     controller_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(dsr_controller2_share, 'launch', 'dsr_controller2.launch.py')
#         ),
#         launch_arguments={'model': LaunchConfiguration('model'),
#                           'color': LaunchConfiguration('color'),
#                           'name': LaunchConfiguration('name')}.items()
#     )

#     # Optionally, include the hardware interface bringup launch file if it exists.
#     # Uncomment and adjust the following if needed:
#     #
#     bringup_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(dsr_bringup2_share, 'launch', 'dsr_bringup2.launch.py')
#         ),
#         launch_arguments={'model': LaunchConfiguration('model'),
#                           'color': LaunchConfiguration('color'),
#                           'name': LaunchConfiguration('name')}.items()
#     )

#     # Launch your custom r_controller node
#     r_controller_node = Node(
#         package='hum_pkg',
#         executable='r_controller',
#         namespace=LaunchConfiguration('name'),
#         output='screen'
#     )

#     # Build the launch description
#     ld = LaunchDescription([
#         model_arg,
#         color_arg,
#         name_arg,
#         description_launch,
#         controller_launch,
#         # bringup_launch,  # Uncomment if you have a bringup launch file for the hardware interface.
#         r_controller_node,
#     ])
#     return ld


# import os
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

# def generate_launch_description():
#     pkg_bringup = get_package_share_directory('dsr_bringup2')
#     pkg_controller = get_package_share_directory('dsr_controller2')
#     pkg_description = get_package_share_directory('dsr_description2')
#     pkg_hardware = get_package_share_directory('dsr_hardware2')
    
#     # Attempt to include dsr_moveit2 if available; otherwise, skip it.
#     try:
#         pkg_moveit = get_package_share_directory('dsr_moveit2')
#         moveit_include = IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(os.path.join(pkg_moveit, 'launch', 'dsr_moveit2.launch.py'))
#         )
#     except PackageNotFoundError:
#         moveit_include = None

#     # Build a list of launch entities.
#     launch_entities = [
#         # Hardware Interface
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(os.path.join(pkg_hardware, 'launch', 'dsr_hardware2.launch.py'))
#         ),
#         # Robot Description (URDF and RViz)
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(os.path.join(pkg_description, 'launch', 'dsr_description.launch.py'))
#         ),
#         # Controllers
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(os.path.join(pkg_controller, 'launch', 'dsr_controller2.launch.py'))
#         ),
#         # RViz for visualization (optional)
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'dsr_bringup2_rviz.launch.py'))
#         ),
#     ]

#     # If MoveIt is available, insert its launch file.
#     if moveit_include is not None:
#         # Insert MoveIt at an appropriate point in the launch sequence.
#         launch_entities.insert(3, moveit_include)

#     return LaunchDescription(launch_entities)

# if __name__ == '__main__':
#     generate_launch_description()


import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch_ros.actions import Node

# movement_node = Node(
#     package='hum_pkg',  # your package containing r_controller
#     executable='r_controller',
#     namespace=LaunchConfiguration('name'),
#     output='screen'
# )

def generate_launch_description():
    # Get share directories for the relevant packages
    pkg_bringup    = get_package_share_directory('dsr_bringup2')
    pkg_controller = get_package_share_directory('dsr_controller2')
    pkg_description= get_package_share_directory('dsr_description2')
    
    # Attempt to include MoveIt if available
    try:
        pkg_moveit = get_package_share_directory('dsr_moveit2')
        moveit_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_moveit, 'launch', 'dsr_moveit2.launch.py'))
        )
    except PackageNotFoundError:
        moveit_include = None

    # Build a list of launch entities.
    launch_entities = [
        # Robot Description (loads the URDF and sets up RViz config, etc.)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_description, 'launch', 'dsr_description.launch.py'))
        ),
        # Controllers (this should start the controller manager and spawn the hardware interface node)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_controller, 'launch', 'dsr_controller2.launch.py'))
        ),
        # RViz Bringup (optional visualization)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'dsr_bringup2_rviz.launch.py'))
        ),
    ]

    # If MoveIt is available, insert its launch file (for motion planning)
    if moveit_include is not None:
        # Insert MoveIt before the RViz bringup
        launch_entities.insert(2, moveit_include)

    return LaunchDescription(launch_entities)  # + [movement_node]

if __name__ == '__main__':
    generate_launch_description()
