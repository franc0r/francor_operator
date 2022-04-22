import os

# from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

  config = os.path.join(
        get_package_share_directory('francor_joy2vel'),
        'param',
        'default.yaml'
        )

  sim_joy2vel_node = Node(package='francor_joy2vel',
                          namespace='',
                          executable='francor_joy2vel_node',
                          name='francor_joy2vel_node',
                          output='screen',
                          parameters=[config],
                          remappings=[
                            #pub
                            ('/cmd_vel', '/francor_frank_base/cmd_vel'),
                            ('/cmd_vel/stamped', '/cmd_vel/stamped'),
                            ('/servo_lx16a/sensor_head_yaw/speed', '/servo_lx16a/sensor_head_yaw/speed'),
                            ('/servo_lx16a/sensor_head_pitch/speed', '/servo_lx16a/sensor_head_pitch/speed'),
                            ('/servo_lx16a/sensor_head_yaw/pos', '/servo_lx16a/sensor_head_yaw/pos'),
                            ('/servo_lx16a/sensor_head_pitch/pos', '/servo_lx16a/sensor_head_pitch/pos'),
                            ('manipulator/speed/axis', 'manipulator/speed/axis'),
                            ('manipulator/speed/inverse', 'manipulator/speed/inverse'),
                            ('francor/add_victim', 'francor/add_victim'),
                            #sub
                            ('/joy', '/joy'),
                            ('/diagnostics', '/diagnostics'),
                            #srv
                            ('/mux/select', '/mux/select'),
                            ('manipulator/set_mode/axis', 'manipulator/set_mode/axis'),
                            ('manipulator/set_mode/inverse', 'manipulator/set_mode/inverse'),
                          ]
                          )

  joy_node = Node(package='joy',
                  namespace='',
                  executable='joy_node',
                  name='joy_node'
                  )


  return LaunchDescription([
    sim_joy2vel_node,
    joy_node
  ])