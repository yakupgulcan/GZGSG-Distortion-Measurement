Tübitak Uzay GZGSG

🇹🇷 [Türkçe](#türkçe) &nbsp;|&nbsp; 🇬🇧 [English](#english)

---

## Türkçe

# GZGSG Distorsiyon Ölçümü ve Kamera Kalibrasyonu

Bu depo, lazer donanımlı bir gimbal (pan-tilt) sistemi kullanarak yüksek doğrulukta kamera kalibrasyonu ve lens distorsiyon ölçümü gerçekleştiren gelişmiş bir **opto-mekanik kalibrasyon sistemi** içerir. Sistem; kameranın içsel parametrelerini ($f_x, f_y, c_x, c_y$) ve distorsiyon katsayılarını (radyal $k_1, k_2$ ve tanjansiyel $p_1, p_2$) yüksek hassasiyetle çıkarmak amacıyla tasarlanmıştır. Kamera, sabit bir gimbal üzerine lazerle paralel şekilde monte edilir; gimbal bilinen açısal adımlarla hareket ettirilirken, kamera her adımda lazer noktasının görüntü düzlemindeki konumunu kaydeder. Bu şekilde, gerçek açı ile ölçülen piksel konumu arasındaki ilişki doğrudan ölçülmüş olur ve bu ilişkiden lensin distorsiyon davranışı geri çıkarılır.

Bu proje, özellikle optik fıçı (barrel) veya yastık (pincushion) tipi distorsiyonun tamamen ortadan kaldırılmasının kritik önem taşıdığı sistemler için değerlidir: **star tracker** (yıldız izleyici) sistemleri, bilgisayarla görü (computer vision) tabanlı konumlandırma sistemleri ve havacılık/savunma sanayii görüntüleme uygulamaları.

## 🚀 Özellikler

- **Otomatik Gimbal Taraması:** İstenilen boyutta ($N \times M$) ızgara tarama rotası otomatik olarak oluşturulur ve baştan sona kullanıcı müdahalesi gerektirmeden yürütülür. Izgara boyutu (`GRID_SIZE_X`, `GRID_SIZE_Y`) ve fiziksel sınırlar (`SAFE_LIMIT_X`, `SAFE_LIMIT_Y`) kod başında kolayca ayarlanabilir.
- **Backlash'e (Boşluk Payına) Dayanıklı Navigasyon:** Mekanik histerezi ve titreşimi (jitter) tamamen ortadan kaldırmak amacıyla tek yönlü (daktilo tarzı) tarama algoritması ve dinamik bekleme (dwell) süreleri kullanılır.
- **Adaptif Izgara Takibi (AdaptiveGridTracker):** Kameranın fiziksel konumu değişse bile lazer noktasını doğru takip eden akıllı ROI sistemi. Üç kademeli tahmin stratejisi: (1) aynı satırda önceki sütundan ilerle, (2) yeni satırda önceki satırın Y ortalamasını baz al, (3) ilk noktayı çıpa olarak kilitle.
- **Aydınlık Ortam Desteği ve Global Fallback:** Ortam ışığı olan koşullarda da çalışır. Noktanın yerel ROI'de kaçırıldığı durumlarda tüm görüntüde lazer beneğini otomatik bulan ve arama kutusunu lazerin üstüne kilitleyen global fallback mekanizması içerir.
- **NOT_DETECTED Koruma Sistemi:** Lazer noktası bulunamazsa o nokta `NOT_DETECTED` olarak işaretlenip atlanır; saçma bir koordinatın tüm kalibrasyon hesabını bozması engellenir. Fit için minimum 8 geçerli nokta zorunludur.
- **Subpiksel Hassasiyette Merkez Tespiti:** Gauss yumuşatma, arka plan çıkarma, bağlı bileşen analizi ve yoğunluk ağırlıklı ağırlık merkezi (centroid) hesabı ile lazer noktası subpiksel doğrulukta konumlandırılır.
- **Başlangıç Pozlama Kalibrasyonu:** Tarama öncesi tek seferlik otomatik pozlama optimizasyonu yapılır ve bu değer tarama boyunca sabit tutulur; her adımda tekrar bekleme gerekmez.
- **Aykırı Değer (Outlier) Korumalı Fit:** Soft-L1 kayıp fonksiyonuyla dirençli optimizasyon, ardından medyan tabanlı istatistiksel aykırı değer eleme; temizlenmiş nokta seti üzerinde Levenberg–Marquardt ile final kalibrasyon yapılır.
- **Kapalı Döngü Senkronizasyon:** Python betiği `M400` ve `M114` G-code komutlarıyla Marlin Firmware ile senkronize çalışır; hareket halindeyken çekilen bulanık kareler veri setine karışmaz.
- **Otomatik Görselleştirmeler:** *İdeal (distorsiyonsuz)* ızgara ile *gerçek (ölçülen)* ızgaranın üst üste bindirildiği birleşik görseller ve CSV çıktıları otomatik olarak üretilir.

## Donanım Gereksinimleri

- **Kamera:** Allied Vision (Vimba SDK) veya OpenCV ile uyumlu herhangi bir endüstriyel kamera.
- **Gimbal / Pan-Tilt Ünitesi:** Marlin Firmware çalıştıran, step motor tahrikli gimbal (Seri USB üzerinden kontrol).
- **Lazer:** Kameraya paralel şekilde monte edilmiş, odaklanabilir lazer işaretleyici.

## Sistem Mimarisi ve Matematiği

Sistem, gimbal'in fiziksel açılarını (azimut ve elevasyon) kameranın görüntü düzlemine üç adımda eşler:

1. **Fiziksel Açıdan Normalize Görüntü Koordinatlarına:**

   $x' = \tan(\theta_{azimut})$
   $y' = \tan(\theta_{elevasyon})$

2. **Distorsiyon Modeli (Brown–Conrady):**

   $r^2 = (x')^2 + (y')^2$

   $x_{dist} = x' \cdot (1 + k_1 r^2 + k_2 r^4) + 2 p_1 x' y' + p_2 (r^2 + 2(x')^2)$

   $y_{dist} = y' \cdot (1 + k_1 r^2 + k_2 r^4) + p_1 (r^2 + 2(y')^2) + 2 p_2 x' y'$

3. **Piksele Projeksiyon:**

   $x_{piksel} = f_x \cdot x_{dist} + c_x$
   $y_{piksel} = f_y \cdot y_{dist} + c_y$

Betik, ölçülen lazer piksel koordinatlarını `scipy.optimize.least_squares` fonksiyonuna besleyerek $f_x, f_y, c_x, c_y, k_1, k_2, p_1, p_2$ değerlerini tersine mühendislikle bulur.

## Kullanım

1. Lazeri ve kamerayı gimbal üzerine monte edin ve düz bir projeksiyon duvarına yönlendirin.
2. `laser_grid_scan.py` dosyası içindeki `PORT` ve `BAUD` ayarlarını donanımınıza göre güncelleyin.
3. İstediğiniz ızgara boyutunu ve fiziksel sınırları ayarlayın:
   ```python
   GRID_SIZE_X = 7      # X eksenindeki nokta sayısı
   GRID_SIZE_Y = 7      # Y eksenindeki nokta sayısı
   SAFE_LIMIT_X = 80.0  # Güvenli X sınırı (mm)
   SAFE_LIMIT_Y = 30.0  # Güvenli Y sınırı (mm)
   ```
4. Betiği çalıştırın:
   ```bash
   python laser_grid_scan.py
   ```
5. Betik otomatik olarak zaman damgalı bir klasör oluşturur (örn. `olcum_7x7_tarih_...`) ve şunları içerir:
   - Yakalanan ham `.png` görüntü kareleri
   - Ham koordinat eşlemelerini içeren `sonuclar.csv`
   - Nihai kalibrasyon parametrelerini içeren `kalibrasyon_sonuc.csv`
   - Distorsiyon eğrisini gösteren `barrel_pincushion.png`
   - Ölçülen ızgara mesh'ini gösteren `composite_mesh.png`

## OpenCV Windows Hatası Üzerine Not

Betik, Windows'ta OpenCV'nin Unicode (ASCII dışı) karakter içeren klasör adlarına görüntü kaydedememesi hatasını aşmak için özel bir `cv2_imwrite_utf8` sarmalayıcı fonksiyon içerir.

## 📄 Raporlar ve Dokümantasyon

- **`Gimbal_Distorsiyon_Detaylı_Proje_Raporu.docx`** — Projenin tam ve güncel raporu. Checkerboard tabanlı başlangıç kalibrasyonunu ve gimbal + lazer tarama yöntemini uçtan uca belgeler.
- **`Checkerboard_Kalibrasyon_Sunum.pptx`** — Gimbal tabanlı sisteme geçilmeden önce kullanılan klasik satranç tahtası (Zhang yöntemi) kalibrasyonunu anlatan sunum.
- **`Gimbal_Kalibrasyon_genel_perspektif_Sunum.pptx`** — Gimbal tabanlı distorsiyon ölçüm düzeneğinin genel hatlarını anlatan sunum: kullanılan cihazlar, alınan ölçümler ve öneriler.
- **`HeNe_lazer_sonrası_ve_karsilasilan_hatalar.pptx`** — Sistemsel hatanın olası kaynakları, yapılabilecek değişiklikler ve HeNe lazerine geçiş. Hazırlayanlar: Ece Salman & Ekin Naz Partal.
- **`HeNe_Fiber_Kuplaj_Teknik_Rapor.pdf`** — HeNe 30989 ile fiber optik kuplaj ve ışın profillerinin (Gaussian, top-hat, donut) incelenmesi; M44L01 (multimode) ve P1-630A-FC-2 (single-mode) fiber karşılaştırması ve laboratuvar uygulama planı.
- **`gimbal_system_visualization.py`** — Gimbal'in hareket aralığını 3 boyutlu olarak çizen prototip betik.

---

*Yüksek hassasiyetli Elektro-Optik Mühendisliği ve Ar-Ge uygulamaları için geliştirilmiştir.*

---

## English

# GZGSG Distortion Measurement & Camera Calibration

This repository contains an advanced **Opto-Mechanical Camera Calibration and Lens Distortion Measurement** system. It is designed to extract highly accurate intrinsic camera parameters ($f_x, f_y, c_x, c_y$) and distortion coefficients (Radial $k_1, k_2$ and Tangential $p_1, p_2$) using a laser-equipped gimbal system.

This project is particularly useful for high-precision optical systems such as **Star Trackers**, Computer Vision localization systems, and aerospace/defense imaging applications where eliminating optical barrel/pincushion distortion is mission-critical.

## 🚀 Features

- **Automated Gimbal Scanning:** Generates and executes a configurable $N \times M$ grid scanning path with no user intervention. Grid size (`GRID_SIZE_X`, `GRID_SIZE_Y`) and physical limits (`SAFE_LIMIT_X`, `SAFE_LIMIT_Y`) are easily set at the top of the script.
- **Anti-Backlash Navigation:** Uses a unidirectional (typewriter-style) scanning algorithm and dynamic dwell times to completely eliminate mechanical hysteresis and vibration (jitter).
- **Adaptive Grid Tracker (AdaptiveGridTracker):** Smart ROI system that correctly tracks the laser dot even when the camera's physical position changes. Three-level prediction strategy: (1) advance from previous column in same row, (2) use previous row's average Y for new row start, (3) lock first detected point as anchor.
- **Bright Environment Support & Global Fallback:** Works even under ambient lighting. Includes a global fallback mechanism that searches the entire image for the laser dot when the local ROI misses it, then locks the search box onto the laser.
- **NOT_DETECTED Protection:** If the laser dot cannot be found, the point is marked `NOT_DETECTED` and skipped — a bad coordinate never corrupts the calibration fit. Minimum 8 valid points required for fit.
- **Sub-Pixel Centroiding:** Gaussian smoothing, background subtraction, connected component analysis, and intensity-weighted centroid for sub-pixel accurate laser dot localization.
- **One-Shot Exposure Calibration:** Exposure is auto-optimized once before scanning begins and locked for the entire scan — no per-step waiting or retries.
- **Outlier-Robust Fit:** Soft-L1 loss robust optimization followed by median-based statistical outlier rejection; final calibration via Levenberg–Marquardt on the cleaned point set.
- **Closed-Loop Synchronization:** Flawlessly syncs Python with Marlin Firmware (G-Code) using `M400` and `M114` to ensure images are captured only when physical movement has completely stopped.
- **Automated Visualizations:** Generates composite mesh images overlaying the *ideal (undistorted)* grid with the *actual (measured)* grid, plus CSV outputs.

## Hardware Requirements

- **Camera:** Allied Vision (Vimba SDK) or any OpenCV-compatible industrial camera.
- **Gimbal/Pan-Tilt Unit:** Stepper-motor driven gimbal running Marlin Firmware (controlled via Serial USB).
- **Laser:** A focusable laser pointer mounted parallel to the camera.

## System Architecture & Mathematics

The system maps the physical angles of the gimbal (Azimuth & Elevation) into the camera's image plane.

1. **Physical Angle to Normalized Image Coordinates:**

   $x' = \tan(\theta_{azimuth})$
   $y' = \tan(\theta_{elevation})$

2. **Distortion Model (Brown-Conrady):**

   $r^2 = (x')^2 + (y')^2$

   $x_{dist} = x' \cdot (1 + k_1 r^2 + k_2 r^4) + 2 p_1 x' y' + p_2 (r^2 + 2(x')^2)$

   $y_{dist} = y' \cdot (1 + k_1 r^2 + k_2 r^4) + p_1 (r^2 + 2(y')^2) + 2 p_2 x' y'$

3. **Projection to Pixels:**

   $x_{pixel} = f_x \cdot x_{dist} + c_x$
   $y_{pixel} = f_y \cdot y_{dist} + c_y$

The script feeds the measured laser pixel coordinates into `scipy.optimize.least_squares` to reverse-engineer the variables ($f_x, f_y, c_x, c_y, k_1, k_2, p_1, p_2$).

## Usage

1. Mount the laser and camera on the gimbal and face a flat projection wall.
2. Update the `PORT` and `BAUD` settings in `laser_grid_scan.py`.
3. Set your desired grid size and physical limits:
   ```python
   GRID_SIZE_X = 7      # Number of points along X
   GRID_SIZE_Y = 7      # Number of points along Y
   SAFE_LIMIT_X = 80.0  # Safe X boundary (mm)
   SAFE_LIMIT_Y = 30.0  # Safe Y boundary (mm)
   ```
4. Run the script:
   ```bash
   python laser_grid_scan.py
   ```
5. The script will automatically create a uniquely timestamped directory (e.g., `olcum_7x7_tarih_...`) containing:
   - Captured raw `.png` frames
   - `sonuclar.csv` with raw coordinate mappings
   - `kalibrasyon_sonuc.csv` with the final intrinsic matrix and distortion array
   - `barrel_pincushion.png` visually showing the distortion curve
   - `composite_mesh.png` showing the measured grid mesh

## Notes on OpenCV Windows Bug

The script includes a custom `cv2_imwrite_utf8` wrapper function to bypass a known OpenCV C++ bug on Windows that prevents saving images to directories containing Unicode (Non-ASCII) characters.

## 📄 Reports & Documentation

- **`Gimbal_Distorsiyon_Detaylı_Proje_Raporu.docx`** — The full, up-to-date project report documenting the complete calibration methodology end-to-end.
- **`Checkerboard_Kalibrasyon_Sunum.pptx`** — Presentation covering the classical checkerboard (Zhang's method) calibration used before the gimbal-based system.
- **`Gimbal_Kalibrasyon_genel_perspektif_Sunum.pptx`** — General overview of the gimbal-based distortion measurement setup.
- **`HeNe_lazer_sonrası_ve_karsilasilan_hatalar.pptx`** — Systematic error analysis and the switch to a HeNe laser. Prepared by Ece Salman & Ekin Naz Partal.
- **`HeNe_Fiber_Kuplaj_Teknik_Rapor.pdf`** — Technical report on HeNe 30989 fiber optical coupling and beam profile characterization (Gaussian, top-hat, donut modes), comparing M44L01 multimode and P1-630A-FC-2 single-mode fibers with laboratory implementation plan.
- **`gimbal_system_visualization.py`** — A prototype script that plots the gimbal's 3D motion range.

---

*Developed for high-precision Electro-Optical Engineering and R&D applications*
