import serial
import time
import csv
import os
from datetime import datetime

PORT = "COM9"
BAUD = 57600

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
now = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
OUTPUT_FILE = os.path.join(BASE_DIR, f"flight_{now}.csv")
EVENT_FILE  = os.path.join(BASE_DIR, "events.txt")

# Kolom harus sesuai urutan di main.cpp
COLUMNS = [
    "micros",
    "flip",
    "phase",
    "spf_roll",
    "roll_rlv",
    "roll_abs",
    "roll",
    # "pitch",
    # "yaw",
    "spf_r_rate",
    "gxrs",
    # "gyrs",
    # "gzrs",
    # "error_roll",
    # "error_r_rate",
    "m1",
    "m2", 
    "m3", 
    "m4"
]

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2) # Tunggu serial stabil

print(f"Saving to: {OUTPUT_FILE}")

start_time = time.time()

# Counter untuk flush data
line_count = 0 

with open(OUTPUT_FILE, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(COLUMNS)
    f.flush()  # Pastikan header tertulis

    print("Logging... CTRL+C to stop")
    print("-" * 50)
    print(" | ".join(COLUMNS)) 
    print("-" * 50)

    try:
        while True:
            # Baca baris dan bersihkan whitespace
            try:
                line = ser.readline().decode(errors="ignore").strip()
            except Exception as e:
                print(f"Read Error: {e}")
                continue

            if not line:
                continue

            # Hitung waktu PC saat ini (relatif terhadap start_time)
            # Ini akan dipakai untuk event ARM/DISARM dan pengganti data millis
            current_pc_millis = int((time.time() - start_time) * 1000)

            # 1. Cek Event (ARM/DISARM)
            if line == "ARM" or line == "DISARM":
                print(f"\n>>> [{current_pc_millis} ms] EVENT: {line} <<<\n")
                with open(EVENT_FILE, "a") as ev:
                    ev.write(f"{current_pc_millis},{line}\n")
                continue

            # 2. FILTER HEAD DAN TAIL
            if line.startswith("<") and line.endswith(">"):
                
                # Buang karakter < dan >
                raw_content = line[1:-1].strip()
                
                # Split data berdasarkan spasi
                data = raw_content.split()

                # Validasi jumlah kolom
                if len(data) == len(COLUMNS):
                    
                    # --- MODIFIKASI DISINI ---
                    # Kita timpa data[0] (kolom millis asli Teensy) 
                    # dengan current_pc_millis yang kita hitung di atas.
                    data[0] = str(current_pc_millis)
                    # -------------------------

                    # Tulis ke CSV
                    writer.writerow(data)
                    
                    # Tampilkan ke layar
                    display_text = ", ".join(data) 
                    print(f"[{line_count}] {display_text}")
                    
                    line_count += 1
                    
                    # Flush setiap 10 baris
                    if line_count % 10 == 0:  
                        f.flush()
                        os.fsync(f.fileno())

                else:
                    pass

    except KeyboardInterrupt:
        print("\nStopped by user.")
        
    finally:
        f.flush()
        ser.close()
        print(f"File closed safely. Total lines: {line_count}")