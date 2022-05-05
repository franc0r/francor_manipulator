import os

# from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

  # config = os.path.join(
  #       get_package_share_directory('francor_joy2vel'),
  #       'param',
  #       'default.yaml'
  #       )

  francor_manipulator_node = Node(package='francor_manipulator',
                                  namespace='',
                                  executable='francor_manipulator_node',
                                  name='francor_manipulator_node',
                                  output='screen',
                                  parameters=[],
                                  # arguments=['--ros-args', '--log-level', 'info'],
                                  remappings=[
                                    #pub
                                    # ('/cmd_vel', '/francor_frank_base/cmd_vel'),
                                    # ('/cmd_vel/stamped', '/cmd_vel/stamped'),
                                    # #sub
                                    # #srv
                                    # ('manipulator/set_mode/axis', 'manipulator/set_mode/axis'),
                                    # ('manipulator/set_mode/inverse', 'manipulator/set_mode/inverse'),
                                  ]
                                 ) 

#todo micro-ros-agent?? ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 --baudrate 500000      
  micro_ros_agent = Node(package='micro_ros_agent',
                         executable='micro_ros_agent',
                         name='micro_ros_agent',
                         output='screen',
                         arguments=["serial", "--dev", "/dev/ttyACM0", "--baudrate", "57600"]
                         )

  return LaunchDescription([
    francor_manipulator_node,
    micro_ros_agent
  ])