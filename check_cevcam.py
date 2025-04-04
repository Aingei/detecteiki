import cv2
import numpy as np

# ค่าคงที่
KNOWN_WIDTH = 20.0  # ขนาดจริงของวัตถุ (หน่วย: ซม.)
FOCAL_LENGTH = 500  # ระยะโฟกัสของกล้อง (หน่วย: พิกเซล)

# ฟังก์ชันคำนวณระยะห่าง
def calculate_distance(known_width, focal_length, pixel_width):
    return (known_width * focal_length) / pixel_width

# ฟังก์ชันกำหนดสีและตำแหน่ง
def get_color_and_position(x, frame_width):
    third = frame_width // 3
    if x < third:  # ซ้าย
        return (0, 0, 255), "Left"  # สีแดง
    elif x < 2 * third:  # กลาง
        return (0, 255, 0), "Center"  # สีเขียว
    else:  # ขวา
        return (255, 0, 0), "Right"  # สีน้ำเงิน

# เริ่มต้นกล้อง
cap = cv2.VideoCapture(6)

if not cap.isOpened():
    print("ไม่สามารถเปิดกล้องได้")
    exit()

# ถ่ายภาพอ้างอิง (กด 'r' เพื่อบันทึกภาพพื้นหลัง)
print("กด 'r' เพื่อบันทึกภาพอ้างอิง (ไม่มีวัตถุ)")
while True:
    ret, reference_frame = cap.read()
    if not ret:
        print("ไม่สามารถอ่านภาพจากกล้องได้")
        break
    cv2.imshow("Reference Setup", reference_frame)
    if cv2.waitKey(1) & 0xFF == ord('r'):
        reference_gray = cv2.cvtColor(reference_frame, cv2.COLOR_BGR2GRAY)
        break

# สร้างภาพพื้นหลังสีดำ
frame_height, frame_width = reference_frame.shape[:2]
black_background = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)

# ลูปตรวจจับวัตถุ
while True:
    ret, frame = cap.read()
    if not ret:
        print("ไม่สามารถอ่านภาพจากกล้องได้")
        break

    # ความกว้างของเฟรม
    frame_width = frame.shape[1]

    # แปลงภาพปัจจุบันเป็น grayscale เพื่อเปรียบเทียบ
    current_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # หาความแตกต่างระหว่างภาพอ้างอิงและภาพปัจจุบัน
    diff = cv2.absdiff(reference_gray, current_gray)
    thresh = cv2.threshold(diff, 30, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.dilate(thresh, None, iterations=2)  # ขยายพื้นที่ที่ตรวจจับ

    # หาคอนทัวร์ของวัตถุที่เข้ามาใหม่
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # ใช้ภาพพื้นหลังสีดำเป็นฐาน
    display_frame = black_background.copy()

    for contour in contours:
        if cv2.contourArea(contour) > 1000:  # กรองวัตถุที่มีขนาดใหญ่พอ
            x, y, w, h = cv2.boundingRect(contour)
            pixel_width = w

            # คำนวณระยะห่าง
            distance = calculate_distance(KNOWN_WIDTH, FOCAL_LENGTH, pixel_width)

            # กำหนดสีและตำแหน่ง
            color, position = get_color_and_position(x, frame_width)

            # คัดลอกเฉพาะส่วนของวัตถุจากภาพจริง (RGB) ไปยังภาพพื้นหลังสีดำ
            display_frame[y:y+h, x:x+w] = frame[y:y+h, x:x+w]

            # แสดงระยะห่างและตำแหน่ง
            cv2.putText(display_frame, f"Distance: {distance:.2f} cm, Pos: {position}", 
                        (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    # แสดงผลภาพ
    cv2.imshow("Object Detection", display_frame)

    # ออกเมื่อกด 'q'
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('r'):  # รีเซ็ตภาพอ้างอิง
        reference_gray = current_gray.copy()
        print("บันทึกภาพอ้างอิงใหม่")

# ปิดกล้องและหน้าต่าง
cap.release()
cv2.destroyAllWindows()