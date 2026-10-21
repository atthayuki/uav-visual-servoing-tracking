import cv2
import numpy as np
import time

def main():
    # 1. Inisialisasi Kamera
    cap = cv2.VideoCapture(0)
    
    # Resolusi kamera (standar onboard camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # 2. Kernel untuk membersihkan noise morfologi
    kernel = np.ones((5, 5), np.uint8)

    # Variabel perhitungan FPS
    prev_time = 0
    
    # Batas toleransi "Locked" (dalam piksel dari tengah frame)
    DEADZONE_RADIUS = 35 

    print("====================================================")
    print("  KRTI UAV VISUAL SERVOING & TARGET ACQUISITION")
    print("  Fitur: Target Tracking, Error Vector & Flight Command")
    print("  Tekan 'q' untuk keluar.")
    print("====================================================")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Gagal membaca kamera.")
            break

        # Flip horizontal agar seperti sudut pandang kendali FPV
        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        center_cam_x, center_cam_y = w // 2, h // 2

        # Hitung FPS
        curr_time = time.time()
        fps = 1 / (curr_time - prev_time) if (curr_time - prev_time) > 0 else 0
        prev_time = curr_time

        # Konversi ke HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # -------------------------------------------------------------
        # Filter Warna Target (Contoh: Target Merah Drop Zone)
        # -------------------------------------------------------------
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
        clean_mask = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)

        # Cari Kontur
        contours, _ = cv2.findContours(clean_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Cari target dengan luas terbesar (Prioritas Target Drone)
        best_target = None
        max_area = 0

        for c in contours:
            area = cv2.contourArea(c)
            if area > 1200:  # Threshold noise
                if area > max_area:
                    max_area = area
                    best_target = c

        # -------------------------------------------------------------
        # HUD Visual & Navigasi (Heads-Up Display)
        # -------------------------------------------------------------
        # Lingkaran Deadzone di tengah (Target Lock Zone)
        cv2.circle(frame, (center_cam_x, center_cam_y), DEADZONE_RADIUS, (0, 255, 255), 1)
        # Garis bidik (Crosshair) Drone
        cv2.line(frame, (center_cam_x - 15, center_cam_y), (center_cam_x + 15, center_cam_y), (0, 255, 255), 1)
        cv2.line(frame, (center_cam_x, center_cam_y - 15), (center_cam_x, center_cam_y + 15), (0, 255, 255), 1)

        flight_status = "SEARCHING TARGET..."
        status_color = (0, 165, 255) # Orange

        if best_target is not None:
            bx, by, bw, bh = cv2.boundingRect(best_target)
            target_x = bx + bw // 2
            target_y = by + bh // 2

            # Hitung Error Koordinat terhadap Pusat Kamera Drone
            error_x = target_x - center_cam_x
            error_y = center_cam_y - target_y  # Positif jika target di atas pusat kamera

            # Garis Vektor Kesalahan (Error Vector)
            cv2.line(frame, (center_cam_x, center_cam_y), (target_x, target_y), (0, 0, 255), 2)
            cv2.circle(frame, (target_x, target_y), 6, (0, 0, 255), -1)
            cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)

            # Logika Log Guidance Drone
            distance_to_center = np.sqrt(error_x**2 + error_y**2)
            if distance_to_center <= DEADZONE_RADIUS:
                flight_status = "TARGET LOCKED: READY TO DROP / LAND"
                status_color = (0, 255, 0) # Hijau
            else:
                cmd_x = "RIGHT" if error_x > 0 else "LEFT"
                cmd_y = "FORWARD" if error_y > 0 else "BACKWARD"
                flight_status = f"NAV: CORRECTION [{cmd_x}: {abs(error_x)}px | {cmd_y}: {abs(error_y)}px]"
                status_color = (0, 255, 255) # Kuning

            # Label data telemetry piksel
            cv2.putText(frame, f"Err X: {error_x}px, Err Y: {error_y}px", (bx, by - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # -------------------------------------------------------------
        # Tampilkan OSD (On-Screen Display) Telemetry
        # -------------------------------------------------------------
        cv2.rectangle(frame, (10, 10), (450, 75), (0, 0, 0), -1) # Background bar hitam
        cv2.putText(frame, f"FPS: {fps:.1f} | SENSOR: VIRTUAL CAM", (20, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, f"STATUS: {flight_status}", (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 2)

        # Tampilkan Window
        cv2.imshow("KRTI UAV Target Tracking Terminal", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()  
