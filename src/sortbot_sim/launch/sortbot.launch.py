from ament_index_python.packages import get_package_share_directory as share 
from launch import LaunchDescription 
from launch.actions import (IncludeLaunchDescription,RegisterEventHandler, SetEnvironmentVariable,
                            TimerAction)
from launch.event_handlers import OnProcessExit 
from launch.launch_description_sources import PythonLaunchDescriptionSource 
from launch.substitutions import Command 
from launch_ros.actions import Node 
from launch_ros.parameter_descriptions import ParameterValue 
from moveit_configs_utils import MoveItConfigsBuilder 
from sortbot_sim import world
from os.path import dirname, join 

SIM={'use_sim_time':True}


def generate_launch_description():
    me = share('sortbot_sim')
    xacro,ctrl = join(me,'urdf','sortbot.urdf.xacro'),join(me,'config','ros2_controllers.yaml')
    sdf = '/tmp/sortbot_world.sdf'
    open(sdf,'w').write(world.build())

    desc = ParameterValue(
        Command(['xacro ',xacro,' controllers_file:=',ctrl]),
        value_type=str
    )
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(share('ros_gz_sim'),'launch','gz_sim.launch.py')),
        launch_arguments={'gz_args':f'-r -v2 {sdf}','on_exit_shutdown':'true'}.items())
    rsp=Node(package='robot_state_publisher',executable='robot_state_publisher',
             parameters=[{'robot_description':desc},SIM])
    spawn =Node(package='ros_gz_sim',executable='create',arguments=['-topic','robot_description','-name','panda'])
    bridge = Node(package='ros_gz_bridge',executable='parameter_bridge',arguments=[
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'])

    def spawner(n):
        return Node(package='controller_manager',executable='spawner',arguments=[n],parameters=[SIM])
    jsb,arm,hand = spawner('joint_state_broadcaster'),spawner('panda_arm_controller'),spawner('panda_hand_controller')

    cfg =(MoveItConfigsBuilder('sortbot',package_name='sortbot_sim')
          .robot_description(file_path='urdf/sortbot.urdf.xacro',mappings={'controllers_file':ctrl})
          .robot_description_semantic(file_path='config/sortbot.srdf')
          .robot_description_kinematics(file_path='config/kinematics.yaml')
          .joint_limits(file_path='config/joint_limits.yaml')
          .trajectory_execution(file_path='config/moveit-controllers.yaml')
          .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
          .to_moveit_configs())
    move_group =Node(package='moveit_ros_move_group',executable='move_group',
                     parameters=[cfg.to_dict(),SIM,{'moveit_manage_controllers':False,
                                                    'trajectory_execution.allowed_start_tolerance':0.05,
                                                    'trajectory_execution.allowed_execution_duration_scaling':2.0}])
    def static_tf(args):
        return Node(package='tf2_ros', executable='static_transform_publisher', arguments=args, parameters=[SIM])
    cam_tf = [
        static_tf(['--x','0.5','--y','0.05','--z','1.0','--roll','0','--pitch','1.5708','--yaw','0',
                   '--frame-id','world','--child-frame-id','camera_link']),
        static_tf(['--roll','-1.5708','--pitch','0','--yaw','-1.5708',
                   '--frame-id','camera_link','--child-frame-id','camera_optical_frame']),
    ]
    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',dirname(me)+':'+ dirname(share('moveit_resources_panda_description'))),
        gz,rsp,spawn,bridge,*cam_tf,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb,on_exit=[arm,hand])),
        TimerAction(period=8.0,actions=[
            move_group,
            Node(package='sortbot',executable='detector',parameters=[SIM])]),
        TimerAction(period=12.0,actions=[Node(package='sortbot',executable='sorter',output='screen',parameters=[SIM])]),
    ])
