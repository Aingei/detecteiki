from ultralytics import YOLO
import cv2
import numpy as np

# โหลดโมเดล YOLO11n
model = YOLO("testty.pt")  # ใช้ pretrained model หรือระบุ path ถ้ามีไฟล์ custom

# เปิดกล้อง (0 คือกล้อง default)
cap = cv2.VideoCapture(0)

# ตรวจสอบว่ากล้องเปิดได้หรือไม่
if not cap.isOpened():
    print("ไม่สามารถเปิดกล้องได้")
    exit()

# ลูปสำหรับการประมวลผลแบบ real-time
while True:
    # อ่าน frame จากกล้อง
    ret, frame = cap.read()
    
    # ถ้าอ่าน frame ไม่ได้ ให้ออกจากลูป
    if not ret:
        print("ไม่สามารถรับ frame ได้")
        break
    
    # ทำการตรวจจับด้วย YOLO11n
    results = model(frame)
    
    # วาดผลลัพธ์ลงบน frame
    annotated_frame = results[0].plot()
    
    # แสดง frame ที่มีผลลัพธ์
    cv2.imshow('YOLO11n Real-time Detection', annotated_frame)
    
    # กด 'q' เพื่อออก
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ปิดกล้องและหน้าต่าง
cap.release()
cv2.destroyAllWindows()