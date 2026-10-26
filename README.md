# 🤖 Dual-Mode Autonomous Line Follower & Wall Follower (ESP32 / Arduino)

Dokumentasi repositori untuk program robot navigasi otomatis berbasis mikrokontroler ESP32. Robot ini mengombinasikan algoritma **PID Line Follower** (pembaca jalur garis) dan **PID Wall Follower / Obstacle Avoider** (penghindar rintangan/dinding) yang terintegrasi secara otomatis, dilengkapi dengan sistem penyimpanan memori non-volatile (Preferences/NVRAM).

## 📌 Daftar Isi

* [Fitur Utama](#-fitur-utama)

* [Konfigurasi Pin Hardware](#-konfigurasi-pin-hardware)

* [Ekspektasi Alur Kerja Program](#-ekspektasi-alur-kerja-program)

* [Menu Perintah Serial Interactive](#-menu-perintah-serial-interactive)

* [Detail Logika & Algoritma](#-detail-logika--algoritma)

* [Panduan Penggunaan](#-panduan-penggunaan)

## 🚀 Fitur Utama

1. **Auto-Switching Line & Wall Follower:**
   Robot dapat berpindah secara otomatis dari mode penyusur garis (`AUTO_LINE`) ke mode penyusur dinding/rintangan (`AVOIDER`) saat menemukan halangan di depan, lalu kembali menyusuri garis secara mulus (*smooth recovery*).

2. **Kontrol Terpisah PID (Line & Wall):**

   * **Line PID:** Menjaga presisi robot di atas garis hitam/putih.

   * **Wall PID:** Menjaga jarak konstan (9 cm) dari dinding/rintangan di sisi kiri.

3. **Penyimpanan Permanen (NVRAM via `Preferences.h`):**

   * Nilai threshold kalibrasi sensor garis disimpan permanen.

   * Peta navigasi (*Route Map*) persimpangan disimpan di memori flash sehingga tidak hilang saat direstart.

4. **Failsafe & Blind Duration:**
   Mencegah pembacaan garis palsu saat robot baru saja beralih menghindari rintangan (fitur *Blind Mode* selama 600 ms).

5. **Interactive Serial Monitor Interface:**
   Memungkinkan pengujian manual, debug sensor *real-time*, kalibrasi otomatis, dan pengiriman rute peta via Serial Monitor (Baudrate: `115200`).

## 🛠️ Konfigurasi Pin Hardware

| Komponen | Nama Pin | Pin ESP32 | Keterangan | 
| ----- | ----- | ----- | ----- | 
| **Motor Driver (Kiri)** | PWM Kiri | `GPIO 32` | Kontrol Kecepatan Motor Kiri | 
|  | DIR 1 Kiri | `GPIO 26` | Arah Putar Motor Kiri | 
|  | DIR 2 Kiri | `GPIO 27` | Arah Putar Motor Kiri | 
| **Motor Driver (Kanan)** | PWM Kanan | `GPIO 13` | Kontrol Kecepatan Motor Kanan | 
|  | DIR 1 Kanan | `GPIO 25` | Arah Putar Motor Kanan | 
|  | DIR 2 Kanan | `GPIO 12` | Arah Putar Motor Kanan | 
| **Sensor Garis (TCRT5000)** | Sensor 0 (Pojok Kiri) | `GPIO 14` | Pembacaan Analog | 
|  | Sensor 1 (Kiri) | `GPIO 35` | Pembacaan Analog (Input Only) | 
|  | Sensor 2 (Kanan) | `GPIO 34` | Pembacaan Analog (Input Only) | 
|  | Sensor 3 (Pojok Kanan) | `GPIO 33` | Pembacaan Analog | 
| **Ultrasonik Depan** | Trig Depan | `GPIO 18` | Output Pulsa | 
|  | Echo Depan | `GPIO 19` | Input Pulsa | 
| **Ultrasonik Kiri** | Trig Kiri | `GPIO 16` | Output Pulsa | 
|  | Echo Kiri | `GPIO 17` | Input Pulsa | 

## 🔄 Ekspektasi Alur Kerja Program

```
             +-----------------------+
             |      Inisialisasi     |
             | Memuat Threshold/Map  |
             +-----------+-----------+
                         |
                         v
             +-----------------------+
             |    Mode: AUTO_LINE    | <------------------------+
             | (PID Line Following)  |                          |
             +-----------+-----------+                          |
                         |                                      |
              [Jarak Depan < 16 cm?]                            |
                         |                                      |
                  +------+------+                               |
                YA|             |TIDAK                          |
                  v             v                               |
    +------------------------+  (Lanjut Line Follower)          |
    | Pivot Kanan (260ms)    |                                  |
    | Aktifkan Blind Duration|                                  |
    +-----------+------------+                                  |
                |                                               |
                v                                               |
    +------------------------+                                  |
    |     Mode: AVOIDER      |                                  |
    | (PID Wall Follower Kiri|                                  |
    +-----------+------------+                                  |
                |                                               |
   [Deteksi Garis Kembali? (Setelah Blind Durasi Selesai)]       |
                |                                               |
         +------+------+                                        |
       YA|             |TIDAK                                   |
         v             v                                        |
+------------------+  (Lanjut Nyusur Dinding)                  |
| KembaliKeGaris() |                                            |
| Manuver Pelurusan|--------------------------------------------+
+------------------+

```

1. **Booting/Inisialisasi:**

   * ESP32 membaca data kalibrasi threshold dan rute *map* dari EEPROM/Preferences.

   * Menampilkan menu utama pada Serial Monitor.

2. **Kondisi Normal (`AUTO_LINE`):**

   * Robot menyusuri garis hitam menggunakan 4 sensor TCRT5000 dengan kalkulasi PID (`Kp = 40.0`, `Kd = 70.0`).

   * Jika terdeteksi persimpangan (Sensor 0 & 3 membaca garis bersamaan), robot merespons aksi yang terprogram di `routeMap` (`L` = Belok Kiri, `R` = Belok Kanan, `S` = Lurus).

3. **Deteksi Rintangan di Depan:**

   * Ketika Ultrasonik Depan mendeteksi benda/box pada jarak **< 16 cm**, robot otomatis berhenti sejenak dan melakukan **Pivot Kanan (260ms)**.

   * Status beralih ke mode **`AVOIDER`** dan mengaktifkan pengunci waktu buta garis (`DURASI_BLIND = 600ms`).

4. **Penyusuran Dinding/Box (`AVOIDER`):**

   * Robot menyusuri sisi luar rintangan menggunakan Ultrasonik Kiri dengan target jarak **9 cm** (`Kp_Wall = 7.5`, `Kd_Wall = 110.0`).

   * **Handling Tikungan:**

     * *Tikungan Dalam (Depan < 12cm):* Robot Pivot Kanan.

     * *Tikungan Luar (Kiri > 60cm / Hilang Dinding):* Robot bermanuver putar kiri mengitari sudut rintangan.

5. **Kembali ke Garis (`kembaliKeGaris`):**

   * Setelah masa *blind* berlalu, jika sensor TCRT5000 mendeteksi garis kembali, robot menjalankan fungsi pelurusan (`kembaliKeGaris()`).

   * Robot melakukan interupsi gerakan maju sedikit, memutar posisi hingga sejajar dengan garis, mereset nilai PID error, dan berpindah kembali ke mode **`AUTO_LINE`**.

## 💬 Menu Perintah Serial Interactive

Buka Serial Monitor dengan Baudrate **115200** dan `Newline` diaktifkan. Kirimkan perintah berupa karakter berikut:

| Perintah | Aksi / Fungsi | 
| ----- | ----- | 
| `1` | **Memulai Kalibrasi Auto:** Robot berputar spot selama 5 detik untuk mencari nilai Min/Max sensor garis, lalu otomatis menyimpan nilainya ke NVRAM. | 
| `2` | **Mode Debugging:** Menampilkan nilai mentah (*raw value*) sensor garis dan jarak kedua sensor ultrasonik secara *real-time*. | 
| `3` | **Start Auto Mode:** Menjalankan robot dalam mode penyusur garis otomatis berdasarkan Peta/Map. | 
| `4` | **Start Wall Follower Manual:** Memaksa robot langsung masuk ke mode Avoider/Wall Follower. | 
| `0` | **Stop / Idle:** Menghentikan pergerakan motor DC secara instant. | 
| `MAP:XXXX` | **Input Peta Navigasi:** Menyimpan urutan instruksi persimpangan ke NVRAM.  Example: `MAP:LSRL` *(L=Left, S=Straight, R=Right)* | 

## ⚙️ Detail Algoritma & Formula Kontrol

### 1. Error Mapping Line Follower

Sensor garis dikalkulasikan menjadi status error diskrit berdasarkan pembacaan boolean threshold:

* `Error -3` s/d `-1` : Robot terlalu ke kanan (perlu koreksi kiri).

* `Error 0` : Robot berada pas di tengah garis.

* `Error 1` s/d `3` : Robot terlalu ke kiri (perlu koreksi kanan).

* `Error ±4` : Robot kehilangan garis (*Out of Track*), menggunakan memori `lastError` terakhir.

### 2. PID Wall Follower & Deadzone Filter

Proses penyusuran dinding menggunakan variabel responsif: \$$Error = Target (9\\text{ cm})