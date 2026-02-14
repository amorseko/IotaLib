# Dokumentasi Pin I/O IoTA_Basic

Dokumentasi ini menjelaskan fungsi dari masing-masing pin I/O pada modul IoTA_Basic berdasarkan analisis dari file-file sumber.

## Pin Motor

| Pin | Fungsi | Deskripsi |
|-----|--------|-----------|
| GPIO 18 | PWML1 | Penggerak motor kiri, arah maju |
| GPIO 4 | PWML2 | Penggerak motor kiri, arah mundur |
| GPIO 27 | PWMR1 | Penggerak motor kanan, arah maju |
| GPIO 13 | PWMR2 | Penggerak motor kanan, arah mundur |

## Pin Sensor Garis

| Pin | Fungsi | Deskripsi |
|-----|--------|-----------|
| GPIO 26 | Sensor 1 | Sensor garis paling kiri |
| GPIO 25 | Sensor 2 | Sensor garis kedua dari kiri |
| GPIO 33 | Sensor 3 | Sensor garis ketiga dari kiri |
| GPIO 32 | Sensor 4 | Sensor garis keempat dari kiri |
| GPIO 35 | Sensor 5 | Sensor garis kelima dari kiri |
| GPIO 34 | Sensor 6 | Sensor garis keenam dari kiri |
| GPIO 39 | Sensor 7 | Sensor garis ketujuh dari kiri |
| GPIO 36 | Sensor 8 | Sensor garis paling kanan |

## Pin Tombol

| Pin | Fungsi | Deskripsi |
|-----|--------|-----------|
| GPIO 16 | Tombol PLUS | Tombol penambahan/naik |
| GPIO 15 | Tombol MINUS | Tombol pengurangan/turun |
| GPIO 19 | Tombol BACK | Tombol kembali |
| GPIO 23 | Tombol ENTER | Tombol konfirmasi |

## Pin LED Indikator

| Pin | Fungsi | Deskripsi |
|-----|--------|-----------|
| GPIO 2 | LED | LED indikator status sistem |

## Pin Lainnya

| Pin | Fungsi | Deskripsi |
|-----|--------|-----------|
| SDA (GPIO 21) | Komunikasi I2C | Digunakan untuk komunikasi dengan layar OLED |
| SCL (GPIO 22) | Komunikasi I2C | Digunakan untuk komunikasi dengan layar OLED |
| RST (-1) | Reset OLED | Pin reset untuk layar OLED (tidak digunakan secara aktif) |

## Catatan Tambahan

- Modul ini menggunakan mikrokontroler ESP32 yang memiliki banyak pin GPIO yang dapat dikonfigurasi.
- Pin-pin yang terdaftar di atas adalah pin-pin utama yang digunakan dalam fungsi dasar IoTA_Basic.
- Pin sensor dapat dikonfigurasi ulang melalui fungsi `setPinSensor()` jika diperlukan.
- Pin tombol dapat dikonfigurasi ulang melalui fungsi `setPinTombol()` jika diperlukan.
- Pin motor dapat dikonfigurasi ulang melalui fungsi `setPinMotor()` jika diperlukan.