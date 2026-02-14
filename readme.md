# IoTA Basic - Line Follower Robot Library

**IoTA Basic** adalah pustaka (library) Arduino yang dirancang khusus untuk pengembangan robot *Line Follower* berbasis ESP32. Pustaka ini menyediakan antarmuka yang lengkap untuk mengontrol pergerakan robot, membaca sensor garis, mengatur PID, hingga manajemen memori untuk perencanaan lintasan (Path Planning).

## Fitur Utama

*   **PID Control System**: Implementasi algoritma PID (Proportional-Integral-Derivative) yang stabil dan presisi untuk mengikuti garis.
*   **Path Planning Memory**: Mendukung penyimpanan rencana lintasan (Plan) dan checkpoint (CP) ke dalam EEPROM, memungkinkan robot "mengingat" rute.
*   **Sensor Calibration**: Fitur kalibrasi sensor otomatis dengan nilai referensi yang tersimpan di EEPROM.
*   **OLED Menu System**: Antarmuka menu interaktif menggunakan layar OLED SSD1306 untuk pengaturan parameter tanpa perlu coding ulang (Button Interface).
*   **Motor Control**: Mendukung kontrol motor DC dengan PWM (simetris/asimetris).
*   **OTA Update**: Dukungan pembaruan firmware secara nirkabel (Over-The-Air) melalui WiFi.
*   **WiFi Configuration**: Pengaturan SSID dan Password WiFi yang fleksibel.

## Struktur Project

*   **`src/`**: Berisi kode sumber utama library.
    *   `IotaLib.h`: Header file definisi kelas dan fungsi.
    *   `IoTA_Basic.cpp`: Implementasi logika robot (Source Code).
    *   `libIoTA_Basic.a`: (Optional) Versi pre-compiled/obfuscated dari library ini.
*   **`examples/`**: Contoh penggunaan library (Arduino Sketch).
*   **`build_lib.sh`**: Script utilitas untuk membuat file library statis (`.a`) dari hasil kompilasi Arduino IDE.

## Cara Penggunaan

1.  **Instalasi**: Copy folder ini ke direktori `Documents/Arduino/libraries/`.
2.  **Include**: Tambahkan `#include "IotaLib.h"` di sketsa Arduino Anda.
3.  **Inisialisasi**:
    ```cpp
    IoTA_Basic iota(8); // Inisialisasi dengan 8 sensor
    
    void setup() {
      iota.begin(); 
    }
    
    void loop() {
      iota.homeScreen(); // Menampilkan menu utama
    }
    ```

## Obfuscation (Keamanan Kode)

Project ini dilengkapi dengan script **`build_lib.sh`** yang memungkinkan Anda untuk mengubah *source code* (`.cpp`) menjadi *static library* (`.a`). Hal ini berguna jika Anda ingin mendistribusikan library ini kepada orang lain tanpa memperlihatkan kode aslinya.

**Langkah membuat library terenkripsi:**
1.  Buka sketsa contoh di Arduino IDE & Klik **Verify**.
2.  Jalankan `./src/build_lib.sh` di terminal.
3.  File `libIoTA_Basic.a` akan terbentuk.
