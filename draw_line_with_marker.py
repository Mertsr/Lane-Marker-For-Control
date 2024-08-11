import cv2
import numpy as np
from ultralytics import YOLO
import os

# YOLO model ve görüntü yolları
images_path = "./lanefotodata"
model_path = "/home/merts/ros2_ws/src/simulasyon_2024/scripts/lane.pt"
output_dir = "./deneme_mark"

# Çıktı dizinini oluştur
os.makedirs(output_dir, exist_ok=True)

# YOLO modelini yükle
model = YOLO(model_path)

# Görüntü listesini al
images = os.listdir(images_path)

# Her bir görüntüyü işle
for idx, img_name in enumerate(images):
    # Görüntüyü yükle
    image_path = os.path.join(images_path, img_name)
    image = cv2.imread(image_path)

    # YOLO modeli ile görüntüyü işle
    results = model(image)
    masks = results[0].masks  # Model sonuçlarından maskeleri al

    b_mask = np.zeros(image.shape[:2], dtype=np.uint8)

    # Maskeleri birleştir
    if masks is not None:
        for i, mask in enumerate(masks):
            binary_mask = mask.data.cpu().numpy().astype(np.uint8) * 255

            if len(binary_mask.shape) == 3 and binary_mask.shape[0] == 1:
                binary_mask = binary_mask.squeeze(0)

            if binary_mask is not None and binary_mask.size > 0:
                if binary_mask.shape != b_mask.shape:
                    binary_mask = cv2.resize(binary_mask, (b_mask.shape[1], b_mask.shape[0]))

                b_mask = cv2.bitwise_or(b_mask, binary_mask)

    # Maskeyi kaydet
    output_path = os.path.join(output_dir, f"mask_{idx}.png")
    cv2.imwrite(output_path, b_mask)
    print(f"Saved mask to: {output_path}")

    # Hough Çizgi Dönüşümü kullanarak çizgileri tespit et
    edges = cv2.Canny(b_mask, 50, 150, apertureSize=3)
    lines = cv2.HoughLines(edges, 1, np.pi / 180, 200)

    # Görüntüdeki merkez noktayı belirle
    img_center = b_mask.shape[1] / 2

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
            cv2.line(b_mask, (x1, y1), (x2, y2), (0, 0, 255), 2)

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
    cv2.imshow('Detected Lines', b_mask)
    print(f"Deviation: {deviation}")
    print(f"Steering Angle: {steering_angle}")

    cv2.waitKey(0)

cv2.destroyAllWindows()

