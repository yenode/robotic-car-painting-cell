import os
from pathlib import Path
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def layout_sdf(boxes):
    
    models = []
    for item in boxes:
        size = ' '.join(str(value) for value in item['size'])
        xyz = ' '.join(str(value) for value in item['xyz'])
        color = ' '.join(str(value) for value in item['color'])
        collision = (f'<collision name="collision"><geometry><box><size>{size}</size>'
                     '</box></geometry></collision>') if item['collision'] else ''
        transparency = 1.0-float(item['color'][3])
        models.append(
            f'<model name="{item["id"]}"><static>true</static><pose>{xyz} 0 0 0</pose>'
            f'<link name="body">{collision}<visual name="visual"><geometry><box><size>{size}'
            f'</size></box></geometry><material><diffuse>{color}</diffuse></material>'
            f'<transparency>{transparency}</transparency></visual></link></model>')
    return '\n'.join(models)


def setup(context):
    share = Path(get_package_share_directory('painting_cell'))
    cfg = yaml.safe_load((share/'config/cell.yaml').read_text())
    base = cfg['robot_base']
    description = xacro.process_file(str(share/'urdf/cell.urdf.xacro'), mappings={
        'base_x': str(base[0]), 'base_y': str(base[1]), 'base_z': str(base[2])}).toxml()
    robot = {'robot_description': description}
    semantic = {'robot_description_semantic': (share/'config/painting.srdf').read_text()}
    kinematics = {'robot_description_kinematics': {'ur_manipulator': {
        'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
        'kinematics_solver_search_resolution': 0.005, 'kinematics_solver_timeout': 0.1}}}
    ompl = {'planning_plugins': ['ompl_interface/OMPLPlanner'],
        'request_adapters': ['default_planning_request_adapters/ResolveConstraintFrames',
            'default_planning_request_adapters/ValidateWorkspaceBounds',
            'default_planning_request_adapters/CheckStartStateBounds',
            'default_planning_request_adapters/CheckStartStateCollision'],
        'response_adapters': ['default_planning_response_adapters/AddTimeOptimalParameterization',
            'default_planning_response_adapters/ValidateSolution'],
        'planner_configs': {'RRTConnect': {'type': 'geometric::RRTConnect', 'range': 0.0}},
        'ur_manipulator': {'planner_configs': ['RRTConnect'], 'longest_valid_segment_fraction': 0.005}}
    joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    limits = {'robot_description_planning': {'joint_limits': {j: {
        'has_velocity_limits': True, 'max_velocity': 1.0,
        'has_acceleration_limits': True, 'max_acceleration': 1.0} for j in joints}}}
    moveit = [robot, semantic, kinematics, limits, {'use_sim_time': True,
        'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl', 'ompl': ompl,
        'allow_trajectory_execution': False,
        'moveit_simple_controller_manager': {'controller_names': ['joint_trajectory_controller'],
            'joint_trajectory_controller': {'type': 'FollowJointTrajectory', 'action_ns': 'follow_joint_trajectory',
                'default': True, 'joints': joints}},
        'publish_robot_description_semantic': True,
        'publish_planning_scene': True, 'publish_geometry_updates': True,
        'publish_state_updates': True, 'publish_transforms_updates': True}]
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    # Generate a world from the same layout configuration used for the URDF.
    floor_size = cfg['layout']['overall_floor']
    car_xyz = cfg['car_xyz']
    world = ((share/'worlds/cell.sdf').read_text()
        .replace('FLOOR_SIZE', ' '.join(str(value) for value in floor_size))
        .replace('CAR_POSE', ' '.join(str(value) for value in car_xyz))
        .replace('PEDESTAL_POSE', f'{base[0]} {base[1]} {base[2]/2} 0 0 0')
        .replace('PEDESTAL_SIZE', f'0.45 0.45 {base[2]}')
        .replace('LAYOUT_MODELS', layout_sdf(cfg['layout']['boxes'])))
    runtime = Path(os.environ.get('ROS_LOG_DIR', '/tmp/painting_cell_logs'))
    runtime.mkdir(parents=True, exist_ok=True)
    world_file = runtime/'painting_cell.sdf'
    world_file.write_text(world)
    nodes = [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(share/'models') + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', os.environ.get('LD_LIBRARY_PATH', '') + os.pathsep + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')),
        ExecuteProcess(cmd=['gz', 'sim', '-r'] + ([] if gui else ['-s']) + [str(world_file)], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[robot, {'use_sim_time': True}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
        Node(package='ros_gz_sim', executable='create', arguments=['-name', 'ur20', '-topic', 'robot_description', '-world', 'painting_cell']),
        Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster', 'joint_trajectory_controller', '--controller-manager-timeout', '90']),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen', parameters=moveit)]
    if rviz:
        nodes.append(Node(package='rviz2', executable='rviz2', arguments=['-d', str(share/'config/cell.rviz')], parameters=[robot, semantic, kinematics, {'use_sim_time': True}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'), OpaqueFunction(function=setup)])
