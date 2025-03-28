from ultralytics import YOLO
import cv2

# โหลดโมเดล YOLO
model = YOLO("testty.pt")  # เปลี่ยน path ของไฟล์โมเดลของคุณ

# ระบุ path ของไฟล์รูปภาพ
image_path = "1.jpeg"  # เปลี่ยนเป็น path ของรูปคุณ

# โหลดภาพจากไฟล์
image = cv2.imread(image_path)

# ตรวจสอบว่ารูปโหลดสำเร็จหรือไม่
if image is None:
    print("ไม่สามารถโหลดรูปภาพได้")
    exit()

# กำหนดค่า confidence threshold (ค่าระหว่าง 0 ถึง 1)
confidence_threshold = 0.4  # ตัวอย่าง: การกำหนดค่า confidence เป็น 0.4

# ทำการตรวจจับโดยกำหนดค่า confidence threshold
results = model(image, conf=confidence_threshold)

# วาดผลลัพธ์ลงบนภาพ
annotated_image = results[0].plot()  # 'plot' จะเป็นการวาดกรอบและแสดงผลที่ตรวจจับ

# แสดงผลภาพที่มีการตรวจจับ
cv2.imshow('YOLO11n Detection', annotated_image)
cv2.waitKey(0)  # รอจนกว่าจะกดปุ่มใดๆ
cv2.destroyAllWindows()

# บันทึกผลลัพธ์ (ถ้าต้องการ)
cv2.imwrite("output_image.jpg", annotated_image)
