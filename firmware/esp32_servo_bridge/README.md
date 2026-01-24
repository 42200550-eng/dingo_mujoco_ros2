# ESP32 Servo Bridge Firmware

## Chức năng
- Nhận lệnh 12 khớp qua USB‑UART (binary frame)
- Phát PWM 12 kênh
- Đọc IMU (MPU6050, I2C)
- Safety: về pose an toàn khi mất lệnh

## Thư viện cần cài (Arduino)
- Adafruit_MPU6050
- Adafruit_Sensor

## Cấu hình nhanh
Sửa trong `esp32_servo_bridge.ino`:
- `SERVO_PINS[]` (chân PWM)
- `PWM_MIN_US`, `PWM_MAX_US`
- `MAX_ANGLE_RAD`, `SAFE_POSE_RAD[]`
- `VBAT_PIN`, `VBAT_DIV_RATIO`

## Giao thức UART
PC → ESP:
- Header: 0xAA 0x55
- Payload: seq(1) + 12x int16 (rad * 1000)
- CRC16‑CCITT trên payload

ESP → PC:
- Header: 0x55 0xAA
- Payload: seq(1) + IMU(9x int16) + vbat_mV(int16) + fault(uint8)
- CRC16‑CCITT trên payload
