from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory
def generate_launch_description():
    model_path = os.path.join(
        get_package_share_directory('new_ball_detector'),
        'models',
        'yolov8_center_sys_low.engine' # <-- Change this to the model you want to use
    )
    return LaunchDescription([
        
           
     Node(
            package = 'boosterk1_image_proc',
            executable = 'nv12_converter_node',
            name = 'nv12',
            output = 'screen',
        ),
               
     Node(
            package = 'particle_filter',
            executable = 'bridge_odom',
            name = 'bridge_odom',
            output = 'screen',
        ),
       
     Node(
            package='new_ball_detector',
            executable='ball_detector',
            name='ball_detector',
            parameters=[{
                'show_debug': True,                
                'model_path': model_path
                    }],
            output='screen'
        ),
       
     Node(
            package = 'particle_filter',
            executable = 'map_node',
            name = 'map_node',
            output = 'screen',
        ),
       
     Node(
            package = 'particle_filter',
            executable = 'mcl_node',
            name = 'mcl_node',
            output = 'screen',
        ),
     Node(
        package = 'joint_states_package',
        executable = 'joints_service',
        name = 'joints_service',
    ),

     Node(
        package = 'new_twist_to_k1',
        executable = 'odom_to_tf',
        name = 'odom_to_tf',
    )
     
        ])
