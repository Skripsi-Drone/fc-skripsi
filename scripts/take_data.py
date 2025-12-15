import serial
import time
import csv
from datetime import datetime

# --- KONFIGURASI ---
SERIAL_PORT = 'COM12'  # Sesuaikan dengan port Anda
BAUD_RATE = 57600

# Nama kolom (13 data)
CSV_HEADERS = [
    "Roll", "SP_Roll", "Pitch", "SP_Pitch", "Yaw", "SP_Yaw", 
    "Roll_rate", "Pitch_rate", "Yaw_rate", 
    "m1", "m2", "m3", "m4"
]

def main():
    # Nama file dengan timestamp
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"terbang12Dec_2.csv"

    print(f"Mencoba koneksi ke {SERIAL_PORT}...")
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2) # Tunggu koneksi stabil
        
        with open(filename, mode='w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(CSV_HEADERS) # Tulis header
            
            print(f"Terhubung! Menyimpan data ke: {filename}")
            print("Tekan Ctrl+C untuk berhenti.\n")
            
            while True:
                if ser.in_waiting > 0:
                    try:
                        # Baca baris & hilangkan whitespace
                        raw_line = ser.readline().decode('utf-8', errors='ignore').strip()
                        
                        # Langsung split karena format sudah benar
                        parts = raw_line.split(',')
                        
                        # Validasi: Harus ada 13 data
                        if len(parts) == 13:
                            # Opsional: Convert float untuk memastikan data angka valid
                            # (Bisa dihapus jika ingin performa maksimal raw string)
                            values = [float(x) for x in parts]
                            
                            writer.writerow(values)
                            
                            # Tampilkan monitoring ringkas
                            print(f"LOG >> Roll:{values[0]:5.2f} | Pitch:{values[2]:5.2f} | M1:{values[9]:4.0f}")
                        
                    except ValueError:
                        pass # Abaikan baris kosong/rusak
                        
    except serial.SerialException:
        print(f"Error: Gagal membuka {SERIAL_PORT}.")
    except KeyboardInterrupt:
        print(f"\nSelesai. Data tersimpan di '{filename}'.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()