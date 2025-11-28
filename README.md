<p align="center">
  <img src="header.jpg" alt="header.jpg" width="80%">
</p>

<h1 align="center">🤖 ROS2 + ESP32 Ultrasonic Monitoring System</h1>
<p align="center">
  <i>Project MK Robotika Medis</i>
</p>

---
## 👥 Anggota Kelompok
| Nama            | NIM        |
|-----------------|-----------|
| Dila Fadilatu   | 122430135 |
| Faqih           | 122430135 |
| Angela Bertha   | 122430137 |

---

# 📌 Pendahuluan

Sistem ini merupakan implementasi **ROS2** dengan integrasi **ESP32** sebagai perangkat embedded untuk membaca data sensor, yang disusun untuk memenuhi tugas mata kuliah **Robotika Medis**.

Dalam project ini:
- **ESP32 berperan sebagai *Node Publisher*** yang bertugas mengirimkan data jarak yang dibaca oleh sensor ultrasonik **HC-SR04** melalui protokol **micro-ROS**.
- **Laptop/PC yang menjalankan ROS2 berfungsi sebagai *Node Subscriber*** yang menerima, memproses, dan menampilkan data jarak tersebut secara *real-time*.

Arsitektur ini memungkinkan komunikasi dua arah antara perangkat embedded dan sistem ROS2 melalui jaringan, sehingga data dari sensor dapat langsung dimanfaatkan pada sisi host untuk monitoring maupun pengolahan lebih lanjut.

---

# 🛠️ Langkah-Langkah Pembuatan Sistem  

## 🔧 1. Persiapan Komponen
| No | Komponen | Jumlah |
|----|----------|--------|
| 1 | ESP32 Dev Board | 1 |
| 2 | Sensor HC-SR04 | 1 |
| 3 | Resistor 1.8 kΩ + 3.3 kΩ | 1 set |
| 4 | Breadboard | 1 |
| 5 | Kabel jumper | Beberapa |
| 6 | Kabel USB | 1 |
| 7 | PC Windows + Python + PIXI | 1 |

---
## 🪛 2. Perakitan HC-SR04 dengan ESP32

### Koneksi Pin
| Sensor | ESP32 | Keterangan |
|--------|-------|------------|
| VCC | 5V | Sumber daya |
| GND | GND | Ground |
| Trig | GPIO 15 | Pemicu |
| Echo | GPIO 4 | Lewat resistor divider |

---
### Rangkaian Resistor Divider (Wajib untuk Echo → ESP32)
Tujuan: menurunkan 5V Echo menjadi ~3.3V.

HC-SR04 Echo ---- R1 (1.8 kΩ) ----+----> GPIO 4 ESP32
|
R2 (3.3 kΩ)
|
GND
---

### Program ESP32 (Arduino IDE)
```cpp
#define TRIG_PIN 15
#define ECHO_PIN 4

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  long duration;
  float distance;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2; // cm

  Serial.println(distance); // ROS2 akan baca dari sini
  delay(500);
```
OUTPUT: angka jarak dalam cm via port COM.
---

## 💻 3. Persiapan Workspace PIXI

### **Akses Folder Workspace**

```bash
cd C:\pixi_ws
```

### **Aktifkan Shell PIXI**

```bash
pixi shell
```
Promt akan berubah menjadi:
```powershell
(pixi_ros2_jazzy) PS C:\pixi_ws>
```

### **Masuk ke Workspace ROS2**

```bash
cd C:\pixi_ws\ros2_ws
```
---
## 🚀 4. Membuat Package ROS2
```bash
cd src
ros2 pkg create smart_system --build-type ament_python
```
Struktur terbentuk:
smart_system/
 ├── package.xml
 ├── setup.py
 └── smart_system/
      └── __init__.py

## ⚙️ 5. Pembuatan _Publisher_ dan _Subscriber_  
### **Publisher: publisher_ultrasonic.py**
Membaca data dari COM (ESP32) → mem-publish ke topic ROS2 /distance.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time

class UltrasonicPublisher(Node):
    def __init__(self):
        super().__init__('ultrasonic_publisher')
        self.publisher_ = self.create_publisher(Float32, 'ultrasonic_distance', 10)

        # GANTI PORT sesuai ESP32 kamu
        self.serial_port = serial.Serial('COM9', 115200, timeout=1)

        self.timer = self.create_timer(0.5, self.publish_data)

    def publish_data(self):
        if self.serial_port.in_waiting > 0:
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                distance = float(line)
                msg = Float32()
                msg.data = distance
                self.publisher_.publish(msg)
                self.get_logger().info(f'Distance: {distance} cm')
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
---
### **Subscriber: subscriber_display.py**
Menampilkan nilai jarak di terminal.
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class DisplaySubscriber(Node):
    def __init__(self):
        super().__init__('display_subscriber')

        self.subscription = self.create_subscription(
            Float32,
            'ultrasonic_distance',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Distance received: {msg.data} cm')

def main(args=None):
    rclpy.init(args=args)
    node = DisplaySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
---

## 🖥️ 5. Instalasi Dependency
**Install pyserial**
```bash
pip install pyserial
pip show pyserial
```

***Tambahkan ke setup.py
```phyton
install_requires=['setuptools', 'pyserial'],
```
---

## 🤖 6. Build Workspace ROS2
```bash
cd C:\pixi_ws\ros2_ws
colcon build
. install/setup.ps1
```
---
## 🕹️ 7. Menjalankan Node
### **Menjalankan Publisher**

```bash
ros2 run smart_system publisher_ultrasonic
```

### **Menjalankan Subscriber (Terminal baru)**

```bash
pixi shell
cd C:\pixi_ws\ros2_ws
. install/setup.ps1
ros2 run smart_system subscriber_display
```
Subscriber akan menampilkan data jarak secara _real time_.

## ⛔ 8. Menghentikan Node
- Tekan CTRL + C
- Menutup terminal → otomatis mematikan node
- Jika node macet → hentikan python.exe lewat Task Manager

## 🚧 _Trial & Error_
### **1. Port Arduino tidak terbaca**
**Masalah:**
Device `/dev/ttyUSB0` tidak muncul.

**Solusi:**
- Cek kabel USB  
- Coba ganti port ke `/dev/ttyACM0`  
- Cek daftar port dengan perintah:  
  ```bash
  ls /dev/tty*
  ```
---
### Pertanyaan & Komentar
- Silakan buka `issue` di repositori utama untuk bertanya atau memberi masukan.

<p align="center">
  <b>✨ Terima kasih! ✨</b><br>
</p>

---


