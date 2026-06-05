from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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
            package = 'particle_filter',
            executable = 'detector',
            name = 'detector',
            output = 'screen',
        ),
       
     Node(
            package = 'particle_filter',
            executable = 'mirror_map_node',
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
        package = 'new_twist_to_k1',
        executable = 'odom_to_tf',
        name = 'odom_to_tf',
    )
     
        ])
