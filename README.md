# Lane-Marker-For-Control

Bu proje, **şerit tespiti ve şerit takibi** (lane detection & following) için Python tabanlı görüntü işleme ve kontrol algoritmalarını bir arada sunar. Temel amaç, kamera görüntülerinden şeritleri tespit edip, araç merkezinin yola göre sapmasını hesaplamak ve PID/Oransal kontrol ile otomatik direksiyon komutu üretmektir. YOLO tabanlı segmentasyon, OpenCV tabanlı çizgi tespiti ve ROS2 ile gerçek araç veya simülasyon kontrolü desteklenir.

---

## Özellikler

- **YOLO tabanlı şerit segmentasyonu:** `lane_marker.py`, `draw_lane_with_marker.py`, `lane_following_pid.py`
- **OpenCV ile klasik çizgi tespiti**: `draw_lane.py`
- **Otomatik direksiyon açısı hesaplama:** PID veya oransal kontrol seçeneği
- **Gerçek zamanlı ROS2 entegrasyonu:** `/cmd_vel` mesajı ile hareket komutu
- **Kendi veri setinizle veya kamera görüntüsüyle çalışabilme**
- **Maskeleri kaydetme, görselleştirme ve analiz**

---

## Klasör & Dosya Yapısı

- `lane_following_pid.py`  
  YOLO segmentasyon + PID tabanlı gerçek zamanlı şerit takibi ve ROS2 kontrol düğümü.
- `draw_lane_with_marker.py`  
  YOLO ile tespit edilen maskeler üzerinden çizgi/şerit tespit & görselleştirme.
- `draw_lane.py`  
  Sadece edge/canny ve klasik Hough çizgi tespitiyle şerit analiz örneği.
- `lane_marker.py`  
  YOLO modelinden maske üretip kaydeden toplu işleme scripti.
- `lanefotodata/`  
  Görüntü veri seti klasörü (kendi datasetinizi buraya koyun).
- `deneme_mark/`  
  Çıktı maskelerinin ve sonuçlarının kaydedildiği klasör.

---

## Gereksinimler

- Python 3.8+
- OpenCV (`pip install opencv-python`)
- NumPy
- ultralytics (`pip install ultralytics`) (YOLO için)
- simple-pid (`pip install simple-pid`)
- ROS2 (gerçek zamanlı kontrol için)
- cv_bridge, sensor_msgs, geometry_msgs (ROS2 ortamında)

---

## Kullanım

### 1. Şerit Maskesi Üretimi (YOLO ile)

```bash
python lane_marker.py
```
veya
```bash
python draw_lane_with_marker.py
```
- `lanefotodata/` klasöründeki tüm görüntüler işlenir, şerit maskeleri `deneme_mark/` klasörüne kaydedilir.

### 2. Klasik Görüntü Üzerinde Şerit Analizi

```bash
python draw_lane.py
```
- Tek bir görüntüde kenar tespiti ve Hough çizgi tespiti ile basit analiz yapar ve görselleştirir.

### 3. Gerçek Zamanlı Şerit Takibi ve Araç Kontrolü (ROS2 ile)

```bash
ros2 run <paket_adı> lane_following_pid.py
# ya da
python3 lane_following_pid.py
```
- `/zed_cam/camera_sensor/left/image_raw` topic'indeki görüntüyü işler.
- YOLO ile şerit maskesini çıkarır, çizgileri bulur, sapmayı hesaplarken PID kontrol ile `Twist` mesajı yollar.

---

## Örnek Akış (lane_following_pid.py)

1. **Görüntü alınır → YOLO ile şerit maskesi çıkarılır**
2. **Canny ile kenar bulma → Hough çizgi tespiti**
3. **Sol/sağ çizgilerden yol merkezi ve sapma hesaplanır**
4. **PID kontrol ile direksiyon açısı üretilir**
5. **Direksiyon komutu (`Twist`) yayınlanır ve araç hareket eder**

---

## Notlar

- YOLO model dosya yolunu ve kamera topic isimlerini kendi ortamınıza göre değiştirin.
- Kendi verinizle eğitim/test için `lanefotodata/` klasörünü güncelleyebilirsiniz.

---

## Katkı & Lisans

Katkıda bulunmak için fork'layın ve pull request açın.  
Lisans: [Belirtilmemiş]

---

## İletişim

Proje sahibi: [Mertsr](https://github.com/Mertsr)

---
