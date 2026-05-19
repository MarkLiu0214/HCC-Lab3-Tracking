import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time
import csv
import os

class TelloTrackingNode(Node):
    def __init__(self):
        super().__init__('tello_tracking_node')
        # 訂閱 tello 原始影像
        self.subscription = self.create_subscription(
            Image,
            '/image_raw', 
            self.image_callback,
            10)
        
        # 發布畫上追蹤結果的影像
        self.image_pub = self.create_publisher(Image, '/tracking_result_image', 10)
        
        # 發布 3D 軌跡給 RViz
        self.path_pub = self.create_publisher(Path, '/ball_trajectory', 10)
        
        self.cv_bridge = CvBridge()
        
        # 追蹤與速度計算狀態
        self.prev_center = None
        self.prev_time = None
        
        # --- 3D 空間轉換與軌跡紀錄變數 ---
        self.real_diameter = 0.10  # 球的實際直徑 (公尺)
        self.init_distance = 0.10  # 初始距離 (公尺)
        self.focal_length = None   # 待第一幀畫面進行校正
        
        # 初始化 RViz Path 訊息
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'camera_frame' # RViz 需要一個參考座標系
        
        # 準備存入 CSV 的資料列表
        self.trajectory_data = []  

    def image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'影像轉換失敗: {e}')
            return

        img_h, img_w = cv_image.shape[:2]
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # 紅色遮罩
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            
            if cv2.contourArea(max_contour) > 500:
                x, y, w, h = cv2.boundingRect(max_contour)
                center = (int(x + w / 2), int(y + h / 2))
                
                # 繪製 2D 追蹤框
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(cv_image, center, 5, (255, 0, 0), -1)

                current_time = time.time()
                
                # --- 1. 焦距自動校正 (只在第一幀有效畫面執行) ---
                if self.focal_length is None:
                    # 公式: f = (像素寬度 * 實際距離) / 實際大小
                    self.focal_length = (w * self.init_distance) / self.real_diameter
                    self.get_logger().info(f'完成焦距校正: {self.focal_length:.2f}')
                
                # --- 2. 3D 座標計算 (Pinhole Camera Model) ---
                # 計算深度 Z (前/後)
                Z = (self.real_diameter * self.focal_length) / w
                
                # 相對於影像中心的像素偏移量
                cx_offset = center[0] - (img_w / 2)
                cy_offset = center[1] - (img_h / 2)
                
                # 計算 X (右/左) 與 Y (下/上)
                X = cx_offset * Z / self.focal_length
                Y = cy_offset * Z / self.focal_length
                
                # --- 3. 發布軌跡給 RViz ---
                current_ros_time = self.get_clock().now().to_msg()  # get current ROS time

                pose = PoseStamped()
                pose.header.stamp = current_ros_time
                pose.header.frame_id = 'camera_frame'
                pose.pose.position.x = float(X)
                pose.pose.position.y = float(Y)
                pose.pose.position.z = float(Z)
                
                self.path_msg.header.stamp = current_ros_time  # header of whole path need to update time conti.
                self.path_msg.poses.append(pose)
                self.path_pub.publish(self.path_msg)
                
                # --- 4. 記錄資料準備寫入 CSV ---
                self.trajectory_data.append([current_time, X, Y, Z])
                
                # 顯示 3D 距離資訊在畫面上
                cv2.putText(cv_image, f"Dist(Z): {Z:.2f}m", (x, y - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 100), 2)

                # 計算 2D 像素速度與方向箭頭 (保留原有功能)
                if self.prev_center and self.prev_time:
                    dt = current_time - self.prev_time
                    if dt > 0:
                        dx = center[0] - self.prev_center[0]
                        dy = center[1] - self.prev_center[1]
                        speed = math.sqrt(dx**2 + dy**2) / dt
                        
                        arrow_end = (int(center[0] + dx * 3), int(center[1] + dy * 3))
                        cv2.arrowedLine(cv_image, center, arrow_end, (0, 255, 255), 3, tipLength=0.3)
                        
                        cv2.putText(cv_image, f"Speed: {speed:.1f} px/s", (x, y - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                
                self.prev_center = center
                self.prev_time = current_time
            else:
                self.prev_center = None
                self.prev_time = None
        else:
            self.prev_center = None
            self.prev_time = None

        result_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, 'bgr8')

        result_msg.header.stamp = self.get_clock().now().to_msg()
        result_msg.header.frame_id = 'camera_frame'
        
        self.image_pub.publish(result_msg)

    def destroy_node(self):
        # --- 節點關閉時，將紀錄的軌跡存成 CSV 檔 ---
        save_path = os.path.expanduser('~/ball_trajectory.csv')
        try:
            with open(save_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Timestamp', 'X(m)', 'Y(m)', 'Z_Depth(m)'])
                writer.writerows(self.trajectory_data)
            self.get_logger().info(f'✅ 軌跡已成功儲存至: {save_path}')
        except Exception as e:
            self.get_logger().error(f'軌跡儲存失敗: {e}')
            
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TelloTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass # 允許使用 Ctrl+C 優雅關閉
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
