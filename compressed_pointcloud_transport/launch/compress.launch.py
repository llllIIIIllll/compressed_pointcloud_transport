import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

qomolo_robot_id = os.environ.get("QOMOLO_ROBOT_ID","id")

def generate_launch_description():
   ld = LaunchDescription()

   config = os.path.join(
        get_package_share_directory('compressed_pointcloud_transport'),
        'config',
        'config.yaml'
        )

   node=Node(package='compressed_pointcloud_transport', 
      executable='compress',
      namespace=qomolo_robot_id, 
      name='compress', 
      parameters = [config]
   )

   ld.add_action(node)

   return ld
