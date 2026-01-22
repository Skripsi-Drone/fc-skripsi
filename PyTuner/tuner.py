import serial
import time
import csv
import os
import threading
import msvcrt  # Khusus Windows untuk deteksi keyboard instan
from datetime import datetime

# --- KONFIGURASI ---
PORT = "COM9"       # Sesuaikan Port
BAUD = 57600        # Sesuaikan Baudrate
LOG_DIR = "Data"    # Folder penyimpanan

# Header CSV (Sesuaikan urutan dengan tuningtelem.h kamu)
# Time, Switch, FP, P, D, I, IMAX, MaxRate, PulseDelta, Climb, RateClimb, RateRec, U2, GyroX, Err, Roll
CSV_HEADERS = [
    "millis", "switch_on", "fp", 
    "P", "D", "I", "IMAX", "max_rate",
    "flip_pulse", "climb_ms", "rate_climb", "rate_rec", "u2_max",
    "gyro_x", "error", "roll"
]

# --- SETUP SERIAL ---
try:
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    print(f"Connected to {PORT} at {BAUD}")
except Exception as e:
    print(f"Error opening serial: {e}")
    exit()

# --- SETUP FILE ---
if not os.path.exists(LOG_DIR):
    os.makedirs(LOG_DIR)

now = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
filename = os.path.join(LOG_DIR, f"tuning_flight_{now}.csv")

# Global flags
running = True

# --- FUNGSI MEMBERSIHKAN DATA (CLEANER) ---
def clean_data(raw_items):
    """
    Fungsi ini membuang label teks seperti 'fpd:', 'gx:' 
    sehingga yang masuk ke CSV murni angka.
    Contoh: "fpd:450" -> "450"
    """
    cleaned = []
    for item in raw_items:
        if ":" in item:
            # Ambil bagian setelah titik dua
            cleaned.append(item.split(":")[1])
        else:
            cleaned.append(item)
    return cleaned

# --- THREAD 1: PENERIMA DATA (LOGGER) ---
def read_serial_loop():
    global running
    print(f"Logging to: {filename}")
    
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_HEADERS)
        
        while running:
            try:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    # Cek Framing < ... >
                    if line.startswith("<") and line.endswith(">"):
                        content = line[1:-1] # Buang < dan >
                        
                        # Split koma
                        parts = content.split(",")
                        
                        # Bersihkan label (misal fpd:450 jadi 450)
                        clean_parts = clean_data(parts)
                        
                        # Tulis ke CSV
                        writer.writerow(clean_parts)
                        
                        # Tampilkan ke layar (Opsional: print setiap 5 data biar gak spam)
                        # print(f"Recv: {clean_parts}") 
                        
            except Exception as e:
                print(f"Log Error: {e}")
                
            # Sleep dikit biar CPU gak 100%
            time.sleep(0.001)

# --- THREAD 2: PENGIRIM PERINTAH (TUNER) ---
def user_input_loop():
    global running
    print("-" * 50)
    print("KEYBOARD TUNING ACTIVE")
    print("Tekan tombol tuning (q, a, w, s, r, f, dll).")
    print("Tekan 'ESC' untuk keluar.")
    print("-" * 50)

    while running:
        # Cek apakah ada tombol ditekan (Non-blocking di Windows)
        if msvcrt.kbhit():
            # Baca 1 karakter byte
            key = msvcrt.getch()
            
            # Tombol ESC untuk keluar (ASCII 27)
            if key == b'\x1b':
                print("\nExiting...")
                running = False
                break
            
            # Kirim ke Serial
            try:
                ser.write(key)
                # Print feedback ke layar biar tau kita ngetik apa
                try:
                    print(f" -> Sent Command: {key.decode('utf-8')}")
                except:
                    pass # Abaikan jika tombol aneh
            except Exception as e:
                print(f"Send Error: {e}")

        time.sleep(0.01)

# --- MAIN EXECUTION ---
if __name__ == "__main__":
    # Jalankan Thread Logger (Background)
    t_logger = threading.Thread(target=read_serial_loop)
    t_logger.daemon = True # Mati otomatis kalau program utama mati
    t_logger.start()

    # Jalankan Loop Input (Main Thread)
    try:
        user_input_loop()
    except KeyboardInterrupt:
        running = False

    # Bersih-bersih sebelum tutup
    time.sleep(0.5)
    ser.close()
    print("Serial closed. Data saved.")