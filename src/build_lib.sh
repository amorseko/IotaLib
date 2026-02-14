#!/bin/bash
# Script untuk membuat file library terenkripsi (libIoTA_Basic.a) dari hasil kompilasi Arduino IDE

echo "Mencari hasil kompilasi dari Arduino IDE..."

# Cari file object IoTA_Basic.cpp.o di 5 folder build terbaru
# Kita cari di dalam /Users/muhammadekopriyatna/Library/Caches/arduino/sketches/
# Diurutkan berdasarkan waktu modifikasi (terbaru di atas)

FOUND_OBJECT=""
LATEST_BUILD_DIR=""

# Loop melalui 5 folder terbaru
for dir in $(ls -td /Users/muhammadekopriyatna/Library/Caches/arduino/sketches/* | head -5); do
    if [ -d "$dir" ]; then
        # Coba cari file object di dalam folder ini
        TEMP_OBJ=$(find "$dir" -name "IoTA_Basic.cpp.o" | head -1)
        if [ ! -z "$TEMP_OBJ" ]; then
            FOUND_OBJECT="$TEMP_OBJ"
            LATEST_BUILD_DIR="$dir"
            break
        fi
    fi
done

if [ -z "$FOUND_OBJECT" ]; then
    echo "Error: File IoTA_Basic.cpp.o tidak ditemukan di 5 folder build terbaru."
    echo "Pastikan Anda sudah menekan 'Verify' (Checklist) di Arduino IDE untuk sketsa yang menggunakan library ini."
    exit 1
fi

echo "Ditemukan file object: $FOUND_OBJECT"

# Lokasi tool archiver (xtensa-esp32-elf-ar)
ARCHIVER="/Users/muhammadekopriyatna/Library/Arduino15/packages/esp32/tools/xtensa-esp32-elf-gcc/esp-2021r2-patch5-8.4.0/bin/xtensa-esp32-elf-ar"

if [ ! -f "$ARCHIVER" ]; then
    echo "Error: Tool archiver tidak ditemukan di lokasi: $ARCHIVER"
    exit 1
fi

# Nama file output
OUTPUT_LIB="libIoTA_Basic.a"

echo "Membuat file library statis ($OUTPUT_LIB)..."

# Buat copy sementara object file agar aman
cp "$FOUND_OBJECT" ./IoTA_Basic.o

# Jalankan archiver untuk membuat file .a
"$ARCHIVER" rcs "$OUTPUT_LIB" ./IoTA_Basic.o

# Hapus file object sementara
rm ./IoTA_Basic.o

if [ -f "$OUTPUT_LIB" ]; then
    echo "--------------------------------------------------------"
    echo "SUKSES! File $OUTPUT_LIB berhasil dibuat."
    echo "Lokasi: $(pwd)/$OUTPUT_LIB"
    echo "--------------------------------------------------------"
    echo "Langkah selanjutnya:"
    echo "1. Salin file $OUTPUT_LIB ke: ~/Documents/Arduino/libraries/IotaLib/src/"
    echo "2. Pastikan file IoTA_Basic.cpp SUDAH DIHAPUS di folder tujuan tersebut."
    echo "--------------------------------------------------------"
else
    echo "Gagal membuat file library."
    exit 1
fi
