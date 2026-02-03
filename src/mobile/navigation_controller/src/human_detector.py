#!/usr/bin/env python3

import rospy
import torch
from ultralytics import YOLO
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2

class HumanDetector:
    def __init__(self):
        rospy.init_node("human_detector", anonymous=True)

        # 訂閱相機影像
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

        # 設定發佈最大人員面積
        self.human_area_pub = rospy.Publisher("/human_detection_area", Float32, queue_size=10)

        # 讀取 YOLOv8 模型
        self.model = YOLO("yolov8m.pt")

        # OpenCV 影像轉換橋接
        self.bridge = CvBridge()

        # 存儲最新的最大人員面積
        self.latest_human_area = 0.0

        # 設置 0.5 秒定時器，每 0.5 秒執行一次 `publish_human_area`
        self.timer = rospy.Timer(rospy.Duration(0.5), self.publish_human_area)

        rospy.loginfo("HumanDetector Node Initialized")

    def image_callback(self, msg):
        try:
            # 轉換 ROS 影像到 OpenCV
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            image = cv2.resize(image, (640, 480))

            # 進行物件偵測
            results = self.model(image)

            max_human_area = 0.0  # 只存最大的人員面積

            for result in results:
                for box in result.boxes:
                    cls = int(box.cls[0].item())  # 物件類別
                    conf = box.conf[0].item()  # 信心度

                    if cls == 0 and conf > 0.7:  # 只偵測人 (YOLO class 0)
                        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                        width, height = x2 - x1, y2 - y1
                        area = width * height

                        if area > max_human_area:
                            max_human_area = area  # 只記錄最大面積

                        # 繪製框
                        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # 更新最新的最大人員面積
            self.latest_human_area = max_human_area

            # 顯示影像（可選）
            cv2.imshow("Human Detection", image)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def publish_human_area(self, event):
        """定時發佈最大人員面積"""
        self.human_area_pub.publish(Float32(self.latest_human_area))
        rospy.loginfo(f"Published max human area: {self.latest_human_area}")

if __name__ == "__main__":
    HumanDetector()
    rospy.spin()
