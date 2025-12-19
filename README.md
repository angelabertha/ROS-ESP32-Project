<p align="center">
  <img src="header.png" alt="header.png" width="80%">
</p>

<h1 align="center">🤖 ROS2 + ESP32 Ultrasonic Monitoring System</h1>
<p align="center">
  <i>Project MK Robotika Medis</i>
</p>

---
## 👥 Anggota Kelompok
| Nama                       | NIM        |
|----------------------------|-----------|
| Dila Fadilatu Nisa         | 122430135 |
| Faqih Nurul Haqqi          | 122430125 |
| Angela Bertha M.T          | 122430137 |

---

# 📌 Pendahuluan

Sistem ini merupakan implementasi **ROS2** dengan integrasi **ESP32** sebagai perangkat embedded untuk membaca data sensor, yang disusun untuk memenuhi tugas mata kuliah **Robotika Medis**.

Dalam project ini:
- **ESP32 berperan sebagai *Node Publisher*** yang bertugas mengirimkan data jarak yang dibaca oleh sensor ultrasonik **HC-SR04** melalui protokol **micro-ROS**.
- **Laptop/PC yang menjalankan ROS2 berfungsi sebagai *Node Subscriber*** yang menerima, memproses, dan menampilkan data jarak tersebut secara *real-time*.

Arsitektur ini memungkinkan komunikasi dua arah antara perangkat embedded dan sistem ROS2 melalui jaringan, sehingga data dari sensor dapat langsung dimanfaatkan pada sisi host untuk monitoring maupun pengolahan lebih lanjut.

## 🧩 Diagram Blok Sistem

Diagram blok berikut menunjukkan arsitektur sistem
ROS2 + ESP32 Ultrasonic Monitoring System, mulai dari
sensor, mikrokontroler, hingga komunikasi dengan ROS2.

<p align="center">
  <img src="diagramblok.jpg" alt="Diagram Blok Sistem" width="85%">
</p>


# 🛠️ LANGKAH-LANGKAH PEMBUATAN SISTEM  

## 1 — Persiapan Komponen
| No | Komponen | Jumlah |
|----|----------|--------|
| 1 | ESP32 Dev Board | 1 |
| 2 | Sensor HC-SR04 | 1 |
| 3 | Buzzer | 1 |
| 4 | Breadboard | 1 |
| 5 | Kabel jumper | Beberapa |
| 6 | Kabel USB | 1 |
| 7 | PC Windows + Python + PIXI | 1 |

---
## 2 — Perakitan HC-SR04 dengan ESP32

### Koneksi Pin
| Sensor | ESP32 | 
|--------|-------|
| VCC | 5V | 
| GND | GND | 
| Trig | GPIO 15 |
| Echo | GPIO 4 |

---
### Persiapan ROS
Sebelum melanjutkan ke langkah-langkah selanjutnya, silakan tonton dan ikuti panduan instalasi ROS dan konfigurasi awal pada video berikut:

https://youtu.be/xSXrRQGWmbQ?si=iWGzqBT9VGNmRfPs

video ini menjelaskan secara detail bagaimana cara menyiapkan environtment ROS di Windows agar sesuai dengan versi yang digunakan di panduan ini.

---
### Program ESP32 (Arduino IDE)
cpp
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
}
```
OUTPUT: angka jarak dalam cm via port COM.
---

## 3 — Persiapan Workspace PIXI

### **Akses Folder Workspace**
powershell
```bash
cd C:\pixi_ws
```

### **Aktifkan Shell PIXI**
powershell
```bash
pixi shell
```
Promt akan berubah menjadi:
```powershell
(pixi_ros2_jazzy) PS C:\pixi_ws>
```

### **Masuk ke Workspace ROS2**
powershell
```bash
cd C:\pixi_ws\ros2_ws
```

---
## 4 — Membuat Package ROS2
powershell
```bash
mkdir src
cd src
ros2 pkg create smart_system --build-type ament_python
```
Struktur direktori otomatis terbentuk:
```
smart_system/
  ├── package.xml
  ├── setup.py
  └── smart_system/
       └── __init__.py
```
---

## 5 — Pembuatan _Publisher_ dan _Subscriber_  
### **Publisher: `publisher_ultrasonic.py`**
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
### **Subscriber: `subscriber_display.py`**
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

Note: Pembuatan file publisher dan subscriber dibuat secara manual bisa menggunakan VS Code dengan Type file yaitu Python, selanjutnya file disimpan pada folder dengan struktur direktori seperti di bawah ini:
```
smart_system/
  ├── package.xml
  ├── setup.py
  └── smart_system/
       └── __init__.py
       └── publisher_ultrasonic.py
       └── subscriber_display.py
```
      
## 5 — Instalasi Dependency
**Install pyserial**
powershell
```bash
pip install pyserial
pip show pyserial
```

**Tambahkan ke `setup.py`**
```phyton
install_requires=['setuptools', 'pyserial'],
```
---

## 6 — Build Workspace ROS2
powershell
```bash
cd C:\pixi_ws\ros2_ws
colcon build
. install/setup.ps1
```
---
## 7 — Menjalankan Node
### **Menjalankan Publisher**
powershell
```bash
ros2 run smart_system publisher_ultrasonic
```

### **Menjalankan Subscriber (Terminal baru)**
powershell
```bash
cd C:\pixi_ws\ros2_ws
pixi shell
. install/setup.ps1
ros2 run smart_system subscriber_display

```
Note: Lakukan perlangkah

Subscriber akan menampilkan data jarak secara _real time_.
---

## 8 — Menghentikan Node
- Tekan CTRL + C
- Menutup terminal → otomatis mematikan node
- Jika node macet → hentikan python.exe lewat Task Manager
---

## 🎥 OUTPUT & VIDEO DEMONSTRASI SISTEM

Video berikut menampilkan **output sistem secara real-time**, yang mencakup:
- Pembacaan jarak oleh sensor ultrasonik HC-SR04
- Pengiriman data oleh ESP32 melalui serial
- Publisher ROS2 yang mem-publish data ke topic
- Subscriber ROS2 yang menampilkan data di terminal

### ▶️ Video Output Sistem
[Klik di sini untuk melihat video output sistem](output.mp4)

---
# 🚧 KENDALA & SOLUSI
## **1. Port ESP32 (COM) Berubah-ubah**
**Kendala:**
Setiap kali kabel USB ESP32 dicabut dan dipasang kembali, nomor port COM sering berubah. Akibatnya, ROS2 Publisher tidak bisa membaca data karena kode Python masih menggunakan port lama. Setelah port diubah di kode, workspace juga harus di-build ulang menggunakan:

```
colcon build
. install/setup.ps1
```

**Solusi:**
- Cek ulang nomor port ESP32 di Device Manager setiap kali reconnect.
- Perbarui port di kode Publisher.  
- Build ulang workspace setelah melakukan perubahan.
- (Opsional) Buat script auto-detect COM supaya tidak perlu ganti port manual.

---
## **2. Serial Monitor Arduino Mengunci Port**
**Kendala:**
Saat Serial Monitor Arduino IDE terbuka, port COM milik ESP32 “dikunci” oleh Arduino IDE. ROS2 jadi tidak bisa membaca data dari port tersebut karena hanya satu aplikasi boleh memakai port pada satu waktu.

**Solusi:**
- Tutup Serial Monitor setelah selesai mengecek data dari ESP32.
- Setelah itu jalankan node ROS2 Publisher, agar port tidak konflik dan bisa dibaca oleh ROS2.

---
### Pertanyaan & Komentar
- Silakan buka `issue` di repositori utama untuk bertanya atau memberi masukan.

<p align="center">
  <b>✨ Terima kasih! ✨</b><br>
</p>

---


