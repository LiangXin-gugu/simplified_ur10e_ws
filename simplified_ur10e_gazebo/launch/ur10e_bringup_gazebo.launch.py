# Author: Liang Xin
# Date: Aug 16, 2024
# Description: Launch a robotic arm in Gazebo 
import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():

  # Constants for paths to different files and folders
  package_name_description = 'simplified_ur10e_description'
  package_name_gazebo = 'simplified_ur10e_gazebo'

  default_robot_name = 'ur10e'
  gazebo_launch_file_path = 'launch'
  gazebo_models_path = 'models'
  rviz_config_file_path = 'rviz/ur10e_view_description.rviz'
  urdf_file_path = 'urdf/ur10e_gazebo_launch.xacro'
  world_file_path = 'worlds/pick_place_demo.world'

  # Set the path to different files and folders.  
  pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')  
  pkg_share_description = FindPackageShare(package=package_name_description).find(package_name_description)
  pkg_share_gazebo = FindPackageShare(package=package_name_gazebo).find(package_name_gazebo)
  # FindPackageShare返回package 'mycobot_gazebo' 的share路径，并进一步调用该返回值的find()方法，
  # 返回该路径下的 'mycobot_gazebo' 文件夹所对应的字符串路径。
  # 该字符串路径，可进一步使用os.join方式合成完整路径

  default_rviz_config_path = os.path.join(pkg_share_description, rviz_config_file_path)  
  default_urdf_model_path = os.path.join(pkg_share_gazebo, urdf_file_path)
  gazebo_launch_file_path = os.path.join(pkg_share_gazebo, gazebo_launch_file_path)   
  gazebo_models_path = os.path.join(pkg_share_gazebo, gazebo_models_path)
  world_path = os.path.join(pkg_share_gazebo, world_file_path)
  
  # Launch configuration variables specific to simulation
  robot_name = LaunchConfiguration('robot_name')#声明一个变量robot_name来存放所定义的参数'robot_name'的值
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_rviz = LaunchConfiguration('use_rviz')
  use_sim_time = LaunchConfiguration('use_sim_time')
  world = LaunchConfiguration('world')
  
  # Set the default pose
  x = LaunchConfiguration('x')
  y = LaunchConfiguration('y')
  z = LaunchConfiguration('z')
  roll = LaunchConfiguration('roll')
  pitch = LaunchConfiguration('pitch')
  yaw = LaunchConfiguration('yaw')
  
  # Declare the launch arguments  
  declare_robot_name_cmd = DeclareLaunchArgument(
    name='robot_name',
    default_value=default_robot_name,
    description='The name for the robot') 
  #定义所定义的参数'robot_name'的默认值及相关描述，且能从launch文件外部访问，即调用launch文件时传入该参数值。
  #返回值为句柄，添加到LaunchDescription中。

  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')

  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')

  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='False', #'True','False'
    description='Whether to start RVIZ')
    
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true')

  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')

  declare_x_cmd = DeclareLaunchArgument(
    name='x',
    default_value='0.0',
    description='x component of initial position, meters')

  declare_y_cmd = DeclareLaunchArgument(
    name='y',
    default_value='0.0',
    description='y component of initial position, meters')
    
  declare_z_cmd = DeclareLaunchArgument(
    name='z',
    default_value='0.0',
    description='z component of initial position, meters')
    
  declare_roll_cmd = DeclareLaunchArgument(
    name='roll',
    default_value='0.0',
    description='roll angle of initial orientation, radians')

  declare_pitch_cmd = DeclareLaunchArgument(
    name='pitch',
    default_value='0.0',
    description='pitch angle of initial orientation, radians')

  declare_yaw_cmd = DeclareLaunchArgument(
    name='yaw',
    default_value='0.0',
    description='yaw angle of initial orientation, radians')
    
  # Specify the actions  

  # set gazebo resource path, so that models could be found.
  set_env_vars_resources = AppendEnvironmentVariable(
    'GZ_SIM_RESOURCE_PATH',
    gazebo_models_path)
  
  # Start arm controller
  start_arm_controller_cmd = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
        'arm_controller'],
        output='screen')

  # Start Gazebo environment
  start_gazebo_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    launch_arguments=[('gz_args', [' -r -v 4 ', world])])
  # 加入其他launch文件，ros_gz_sim包中的gz_sim.launch.py文件，启动Gazebo,传入.world文件

  # Start hand controller
  start_hand_controller_cmd =  ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
        'grip_controller'],
        output='screen')

  
  # Launch joint state broadcaster
  start_joint_state_broadcaster_cmd = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
        'joint_state_broadcaster'],
        output='screen')
  # 启动controller：joint_state_broadcaster，该控制器负责往joint_states话题发布数据，发布xacro定义的各关节state_interface的数据。
    
  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  doc = xacro.parse(open(default_urdf_model_path))
  xacro.process_doc(doc)
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{
      'use_sim_time': use_sim_time, 
      'robot_description': doc.toxml()}])
  # 启动robot_state_publisher包中的robot_state_publisher可执行文件，传入参数相关参数。
  # 该节点订阅joint_states话题，即获得关节状态，并据此计算机器人各连杆间的转换关系。
  # 发布robot_description,tf,tf_static三个话题。
  # robot_description话题，其发布机器人的urdf文件，以string形式，相当于实时urdf文件。该话题供rviz2订阅，以实时在rviz2中显示机器人
  # tf,tf_static话题，分别发布动态关节连杆间和静态关节连杆间的转换关系。


  # Launch RViz
  start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])  
    
  # Spawn the robot
  start_gazebo_ros_spawner_cmd = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
      '-string', doc.toxml(),
      '-name', robot_name,
      '-allow_renaming', 'true',
      '-x', x,
      '-y', y,
      '-z', z,
      '-R', roll,
      '-P', pitch,
      '-Y', yaw
      ],
    output='screen')
  # 执行ros_gz_sim中的create可执行文件，相当于在gazebo中spawn一个xacro文件，
  # 该文件中除了包含机器人描述外，还包含gazebo,ros2_control的相关配置。
  # 其中controller_manager就是在这里启动的。

  # Launch the joint state broadcaster after spawning the robot
  load_joint_state_broadcaster_cmd = RegisterEventHandler(
     event_handler=OnProcessExit(
     target_action=start_gazebo_ros_spawner_cmd ,
     on_exit=[start_joint_state_broadcaster_cmd],))

  # Launch the arm controller after launching the joint state broadcaster
  load_arm_controller_cmd = RegisterEventHandler(
    event_handler=OnProcessExit(
    target_action=start_joint_state_broadcaster_cmd,
    on_exit=[start_arm_controller_cmd],))

  # Launch the gripper controller after launching the arm controller
  load_hand_controller_cmd = RegisterEventHandler(
    event_handler=OnProcessExit(
    target_action=start_arm_controller_cmd,
    on_exit=[start_hand_controller_cmd],))

  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_robot_name_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_world_cmd)

  ld.add_action(declare_x_cmd)
  ld.add_action(declare_y_cmd)
  ld.add_action(declare_z_cmd)
  ld.add_action(declare_roll_cmd)
  ld.add_action(declare_pitch_cmd)
  ld.add_action(declare_yaw_cmd)  

  # Add any actions
  ld.add_action(set_env_vars_resources)
  ld.add_action(start_gazebo_cmd)
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_rviz_cmd)
  
  ld.add_action(start_gazebo_ros_spawner_cmd)

  ld.add_action(load_joint_state_broadcaster_cmd)
  ld.add_action(load_arm_controller_cmd) 
  ld.add_action(load_hand_controller_cmd) 

  return ld


