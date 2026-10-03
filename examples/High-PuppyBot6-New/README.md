# High-PuppyBot6 — Arduino IDE

## เริ่มใช้งาน
1. เพิ่ม URL ใน File > Preferences > Additional Boards Manager URLs:
   https://raw.githubusercontent.com/Amornthep-ra/High-PuppyBot6/main/package_highpuppybot6_index.json
2. เปิด Boards Manager แล้วติดตั้ง **High-PuppyBot6**
3. เลือกบอร์ด **High-PuppyBot6** (ตั้ง RP2350B, Flash 4 MB, 150 MHz ให้อัตโนมัติ)
4. ติดตั้ง Adafruit GFX Library, Adafruit ST7735 and ST7789 Library, Adafruit BusIO,
   Adafruit MCP3008, Adafruit TCS34725 และ VL53L0X ของ Pololu ผ่าน Library Manager
5. เปิด High-PuppyBot6-New.ino, เลือกพอร์ตแล้ว Verify/Upload จากนั้นกด SW1 เริ่มงาน

## แก้ไฟล์ไหน
- RobotConfig.h: ค่า RGB, ค่าเส้นดำ, PID, ความเร็ว, ระยะพัลส์, เวลา, ToF และ Pitch
- RobotSensors.h: ตรวจเส้น/สี/ขอบสะพาน และ State
- RobotMotion.h: เดิน ถอย เลี้ยว ขึ้นลงสะพาน
- RobotMission.h: ลำดับค้นหาทาง เริ่มเลี้ยวซ้าย แล้วเลี้ยวขวาเมื่อติดขัด
- High-PuppyBot6-New.ino: setup/loop บนสองคอร์

ไลบรารีสำหรับแก้ไดรเวอร์ในเครื่องนี้:
C:/Users/User/Documents/Arduino/libraries/HighPuppyBot6/src/
แพ็กเกจบอร์ดมีสำเนาไลบรารีเดียวกันรวมมาแล้วด้วย หากแก้ไลบรารีใน Documents ให้ใช้สำเนานั้นเป็นหลัก

## พฤติกรรมที่คงจากโค้ด KBIDE ล่าสุด
- State 0 = เดินครบระยะ/ข้ามอุปสรรค, 1 = เส้นดำ, 2 = ปล่อยสี, 3 = ขอบสะพาน
- ตรวจ ToF ≤ 70 mm ในลูปเดินหลัก; ไม่ได้เพิ่มการตรวจในช่วงเดินต่อ 410 พัลส์หรือคำสั่งเดินแบบครบระยะ
- บนสะพานใช้ Read_status_sensor; พื้นใช้ Threshold ของ TC01
- ลงสะพานแล้วจึงคืน BridgeState=0
- ปล่อยแต่ละสีครั้งเดียว ครบสี่สีหยุด 60 วินาที จากนั้นโค้ดเดิมดำเนินต่อ
- RGB บน SDA0/SCL0 (GPIO4/5); IMU และ ToF บน SDA1/SCL1 (GPIO6/7)
- ไม่มี TCS34725 บน SDA1/SCL1 ตามการต่อจริงที่ยืนยันไว้

เปลี่ยนชื่อตัวแปร/ฟังก์ชันให้อ่านง่าย และเปลี่ยนตัวจับเวลาเป็น uint32_t เพื่อรักษาความละเอียด millis()
ยังต้องทดสอบบนหุ่นยนต์จริงเพื่อยืนยันระยะ แสง และการหยุด
