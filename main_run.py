import rclpy
from rclpy import qos
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2 as cv
import os
from ultralytics import YOLO
from cameracapture import CameraCapture  # ใช้ CameraCapture แทน WindowCapture
import math

# โหลดโมเดล YOLO
model = YOLO('/home/aing/abudetecttest/testty.pt')

# ใช้งานกล้องแทนการจับภาพหน้าจอ
camcap = CameraCapture(camera_index=0)  # ใช้กล้องหลัก (0)

# เปลี่ยนไปที่ไดเร็กทอรีของไฟล์นี้
os.chdir(os.path.dirname(os.path.abspath(__file__)))

class mainRun(Node):
    x: float = 0.0
    y: float = 0.0
    
    def __init__(self):
        super().__init__("Laptop_Node")

        self.sent_where_mob = self.create_publisher(
            Twist, "send_where_mob", qos_profile=qos.qos_profile_system_default
        )
        
        self.sent_data_timer = self.create_timer(0.05, self.sendData)
    
    def detectMobs(self):
        screenshot = camcap.get_screenshot()  # ได้ภาพจากกล้อง (RGB อยู่แล้ว)
        results = model.predict(screenshot)
        
        # ตรวจสอบว่ามีวัตถุถูกตรวจจับหรือไม่
        if len(results[0].boxes) > 0:
            self.x = results[0].boxes.xywh[0][0].item()
            self.y = results[0].boxes.xywh[0][1].item()
        else:
            print("No detections")

        # แสดงภาพที่มีการตรวจจับ
        plot_img = results[0].plot()
        height, width = plot_img.shape[:2]

        # วาดเส้นแบ่งหน้าจอออกเป็น 4 ส่วน
        center_x = width // 2
        center_y = height // 2

        distance = math.sqrt((self.x - center_x) ** 2 + (self.y - center_y) ** 2)


        cv.circle(plot_img, (center_x, center_y), 5, (255, 0, 0), -1)
        cv.circle(plot_img, (int(self.x), int(self.y)), 5, (0, 255, 0), -1)

        # cv.line(plot_img, (part_width, 0), (part_width, height), (0, 255, 0), 2)
        # cv.line(plot_img, (2 * part_width, 0), (2 * part_width, height), (0, 255, 0), 2)
        # cv.line(plot_img, (3 * part_width, 0), (3 * part_width, height), (0, 255, 0), 2)

        cv.putText(plot_img, f"Distance: {distance:.2f}", (50, 50), cv.FONT_HERSHEY_SIMPLEX, 
               0.7, (0, 255, 0), 2)

        cv.imshow('Detection Results', plot_img)
        cv.waitKey(1)
        
    def sendData(self):
        mobdata_msg = Twist()
        self.detectMobs()
        mobdata_msg.linear.x = self.x
        mobdata_msg.linear.y = self.y
        self.sent_where_mob.publish(mobdata_msg)
        
def main():
    rclpy.init()
    sub = mainRun()
    rclpy.spin(sub)
    camcap.release()  # ปิดกล้องเมื่อจบการทำงาน
    rclpy.shutdown()
    cv.destroyAllWindows()
    
if __name__ == "__main__":
    main()
