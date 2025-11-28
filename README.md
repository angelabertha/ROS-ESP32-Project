<p align="center">
  <img src="banner.jpg" alt="banner.jpg" width="80%">
</p>

<h1 align="center">🤖 ROS + Arduino Robot Project</h1>
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

## 📌 Pendahuluan

Dokumentasi ini berisi penjelasan lengkap mengenai project robot berbasis **ROS (Robot Operating System)** dan **Arduino**.  
Project ini dibuat sebagai bagian dari tugas mata kuliah *Robotika Medis*, dengan tujuan mempelajari integrasi antara sistem embedded dan software robotics modern.

Secara umum, project ini akan melibatkan:
- Pembacaan data sensor oleh Arduino  
- Pengiriman data ke ROS melalui komunikasi serial  
- Pengolahan data dalam bentuk node ROS  
- Aksi/output tertentu dari robot
---

## 🧩 Deskripsi Singkat Project

Bagian ini placeholder—silakan kamu ganti nanti sesuai project asli.

Contoh template isi:

Project ini bertujuan membuat robot yang mampu melakukan:
- Mengambil data sensor dari Arduino  
- Mengirimkan data ke ROS dalam bentuk *topic*  
- Mengontrol aktuator melalui node ROS  
- Menampilkan output di RViz atau terminal  

Diagram umum alur kerja (opsional, bisa upload gambar):
[Arduino] → [Serial] → [ROS Node] → [Processing] → [Action/Output]


---

## 🛠️ Langkah-Langkah Pembuatan Sistem  
*(Hardware & Software)*

## 🔧 HARDWARE SETUP
Silakan isi sesuai robotmu. Template contoh:

- Arduino Uno / Nano / Mega  
- Motor driver L298N  
- Sensor (Ultrasonic / IR / Encoder / dsb)  
- Rangka robot + motor DC / servo  
- Power supply  
- Kabel jumper dan prototyping board  

Skema rangkaian bisa ditambahkan di sini:

---
## 💻 SOFTWARE SETUP

### **1. Install ROS**
Lakukan instalasi ROS dengan perintah berikut:

```bash
sudo apt install ros-noetic-desktop-full
```

### **2. Install rosserial**
Rosserial digunakan untuk komunikasi antara Arduino dan ROS.

```bash
sudo apt-get install ros-noetic-rosserial ros-noetic-rosserial-arduino
```

### **3. Membuat Workspace ROS**
Setelah instalasi selesai, buat workspace ROS sebagai tempat penyimpanan package.

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```

### **4. Kode Arduino**
Gunakan kode berikut untuk memastikan komunikasi serial berjalan:

```bash
void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println("hello from arduino");
}
```
## 🚀 Menjalankan Sistem ROS + Arduino
### **1. Menyalakan ROS Master**
ROS Master adalah pusat komunikasi semua node ROS. Jalankan:

```bash
roscore
```
### **2. Menjalankan komunikasi Arduino–ROS**

```bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0
```

### **3. Menjalankan node ROS tambahan (jika ada)**
```bash
rosrun your_package your_node.py
```

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
---

### Pertanyaan & Komentar
- Silakan buka `issue` di repositori utama untuk bertanya atau memberi masukan.

<p align="center">
  <b>✨ Terima kasih! ✨</b><br>
</p>

---


