from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 廣播靜態座標轉換 (TF) -> 這會讓 RViz 自動出現 camera_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_publisher',
            # 參數順序: X Y Z Roll Pitch Yaw 父座標系 子座標系
            # 這裡我們建立一個從 'map' 到 'camera_frame' 的固定座標
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_frame'],
            output='screen'
        ),
        
        # 2. 啟動你寫的影像追蹤 Node
        Node(
            package='tello_tracking',
            executable='tello_tracking_node',
            name='tello_tracking_node',
            output='screen'
        ),
        
        # 3. 啟動系統內建的 RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
