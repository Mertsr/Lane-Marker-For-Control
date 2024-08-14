import cv2
import numpy as np

# Görüntüyü yükle (siyah-beyaz yol şeritlerini içeren görsel)
img = cv2.imread('/mnt/data/deneme_screenshot_11.08.2024.png', 0)

# Kenar tespiti yap (Canny Edge Detection)
edges = cv2.Canny(img, 50, 150, apertureSize=3)

# Hough Çizgi Dönüşümü kullanarak çizgileri tespit et
lines = cv2.HoughLines(edges, 1, np.pi / 180, 200)

# Görüntüdeki merkez noktayı belirle
img_center = img.shape[1] / 2

# Tespit edilen çizgileri depolamak için listeler
left_lines = []
right_lines = []

if lines is not None:
    for line in lines:
        rho, theta = line[0]
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))

        # Çizgiyi görselleştirme (kırmızı renk)
        cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)

        # Çizgileri sola ve sağa göre ayırma
        if x0 < img_center:
            left_lines.append((x1, y1, x2, y2))
        else:
            right_lines.append((x1, y1, x2, y2))

# Ortalama sapmayı hesapla (basit bir ortalama ile)
left_avg_x = np.mean([line[0] for line in left_lines]) if left_lines else img_center
right_avg_x = np.mean([line[0] for line in right_lines]) if right_lines else img_center

# Yolun merkezini hesapla
lane_center = (left_avg_x + right_avg_x) / 2

# Sapmayı hesapla (araç merkezine göre)
deviation = lane_center - img_center

# Direksiyon açısını hesapla (basit oransal kontrol)
k_p = 0.1  # Oransal kazanç değeri
steering_angle = k_p * deviation

# Sonuçları göster
cv2.imshow('Detected Lines', img)
print(f"Deviation: {deviation}")
print(f"Steering Angle: {steering_angle}")

cv2.waitKey(0)
cv2.destroyAllWindows()
