Tübitak Uzay GZGSG

🇹🇷 [Türkçe](#türkçe) &nbsp;|&nbsp; 🇬🇧 [English](#english)

---

## Türkçe

# GZGSG Distorsiyon Ölçümü ve Kamera Kalibrasyonu

Bu depo, lazer donanımlı bir gimbal (pan-tilt) sistemi kullanarak yüksek doğrulukta kamera kalibrasyonu ve lens distorsiyon ölçümü gerçekleştiren gelişmiş bir **opto-mekanik kalibrasyon sistemi** içerir. Sistem; kameranın içsel parametrelerini ($f_x, f_y, c_x, c_y$) ve distorsiyon katsayılarını (radyal $k_1, k_2$ ve tanjansiyel $p_1, p_2$) yüksek hassasiyetle çıkarmak amacıyla tasarlanmıştır. Kamera, sabit bir gimbal üzerine lazerle paralel şekilde monte edilir; gimbal bilinen açısal adımlarla hareket ettirilirken, kamera her adımda lazer noktasının görüntü düzlemindeki konumunu kaydeder. Bu şekilde, gerçek açı ile ölçülen piksel konumu arasındaki ilişki doğrudan ölçülmüş olur ve bu ilişkiden lensin distorsiyon davranışı geri çıkarılır.

Bu proje, özellikle optik fıçı (barrel) veya yastık (pincushion) tipi distorsiyonun tamamen ortadan kaldırılmasının kritik önem taşıdığı sistemler için değerlidir: **star tracker** (yıldız izleyici) sistemleri, bilgisayarla görü (computer vision) tabanlı konumlandırma sistemleri ve havacılık/savunma sanayii görüntüleme uygulamaları gibi yüksek hassasiyet gerektiren optik sistemler. Bu tür sistemlerde küçük bir kalibrasyon hatası bile konumlandırma veya ölçüm doğruluğunda büyük sapmalara yol açabildiğinden, distorsiyonun doğru ve tekrarlanabilir biçimde karakterize edilmesi gerekir.

## 🚀 Özellikler

- **Otomatik Gimbal Taraması:** Yoğun bir $N \times N$ ızgara (örneğin 30×30) tarama rotası otomatik olarak oluşturulur ve baştan sona kullanıcı müdahalesi gerektirmeden yürütülür; her nokta sırasıyla ziyaret edilir ve ölçüm otomatik olarak kaydedilir.
- **Backlash'e (Boşluk Payına) Dayanıklı Navigasyon:** Mekanik histerezi ve titreşimi (jitter) tamamen ortadan kaldırmak amacıyla, satırların her zaman aynı yönde taranmasını sağlayan tek yönlü (daktilo tarzı) bir tarama algoritması ve her hareketten sonra sistemin oturması için dinamik bekleme (dwell) süreleri kullanılır.
- **Subpiksel Hassasiyette Merkez Tespiti:** Lazer noktasının merkezini subpiksel doğrulukta konumlandırmak için gelişmiş bir görüntü işleme hattı uygulanır: Gauss tabanlı saçılma (scattering) bastırma, Otsu eşikleme ve yoğunluk ağırlıklı ağırlık merkezi (centroid) hesabı bu hatta dahildir.
- **Kapalı Döngü Senkronizasyon:** Görüntülerin yalnızca fiziksel hareket tamamen durduktan sonra yakalanmasını garanti etmek için, Python betiği `M400` ve `M114` G-code komutlarıyla Marlin Firmware ile kusursuz biçimde senkronize çalışır; böylece hareket halindeyken çekilen bulanık veya kaymış kareler veri setine karışmaz.
- **Levenberg–Marquardt Optimizasyonu:** Lensin tam distorsiyon dizisini çıkarmak için, fiziksel açı ile piksel konumu arasındaki doğrusal olmayan projeksiyon modeli, Levenberg–Marquardt algoritmasıyla çözülür.
- **Otomatik Görselleştirmeler:** Distorsiyonu (örneğin fıçı/yastık tipini) görsel olarak ortaya koymak amacıyla, *ideal (distorsiyonsuz)* ızgara ile *gerçek (ölçülen)* ızgaranın üst üste bindirildiği birleşik (composite) görseller otomatik olarak üretilir.

## Donanım Gereksinimleri

- **Kamera:** Allied Vision (Vimba SDK) veya OpenCV ile uyumlu herhangi bir endüstriyel kamera kullanılabilir.
- **Gimbal / Pan-Tilt Ünitesi:** Marlin Firmware çalıştıran, step motor tahrikli bir gimbal gereklidir; bu ünite Seri USB üzerinden kontrol edilir.
- **Lazer:** Kameraya paralel şekilde monte edilmiş, odaklanabilir bir lazer işaretleyici kullanılır.

## Sistem Mimarisi ve Matematiği

Sistem, gimbal'in fiziksel açılarını (azimut ve elevasyon) kameranın görüntü düzlemine eşler. Bu eşleme üç adımda gerçekleşir:

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

Betik, ölçülen lazer piksel koordinatlarını ($x_{piksel}, y_{piksel}$) `scipy.optimize.least_squares` fonksiyonuna besleyerek, yukarıdaki modeldeki değişkenleri ($f_x, f_y, c_x, c_y, k_1, k_2, p_1, p_2$) tersine mühendislikle, yani ölçülen verilerden yola çıkarak bulur.

## Kullanım

1. Lazeri ve kamerayı gimbal üzerine monte edin ve düz bir projeksiyon duvarına yönlendirin.
2. `gimbal_laser_calibration.py` dosyası içindeki `PORT` ve `BAUD` ayarlarını, kullanılan donanıma göre güncelleyin.
3. İstediğiniz ızgara boyutunu ayarlayın (örneğin `GRID_SIZE_X = 30`, `GRID_SIZE_Y = 30`).
4. Betiği çalıştırın:
   ```bash
   python gimbal_laser_calibration.py
   ```
5. Betik, çalıştırıldığında otomatik olarak zaman damgalı, benzersiz bir klasör oluşturur (örneğin `olcum_30x30_tarih_...`); bu klasör şunları içerir:
   - Yakalanan ham `.png` görüntü kareleri.
   - Ham koordinat eşlemelerini içeren `sonuclar.csv` dosyası.
   - Nihai içsel parametre matrisini ve distorsiyon dizisini içeren `kalibrasyon_sonuc.csv` dosyası.
   - Distorsiyon eğrisini görsel olarak gösteren `barrel_pincushion.png` dosyası.

## OpenCV Windows Hatası Üzerine Not

Betik, Windows işletim sisteminde OpenCV'nin bilinen bir C++ hatasını aşmak amacıyla özel bir `cv2_imwrite_utf8` sarmalayıcı (wrapper) fonksiyonu içerir. Bu hata, Unicode (ASCII dışı) karakter içeren klasör adlarına (örneğin Türkçe karakterli klasörlere) görüntü kaydedilememesine neden olmaktadır; sarmalayıcı fonksiyon bu sınırlamayı devre dışı bırakır.

## 📄 Raporlar ve Dokümantasyon

- **`Gimbal_Distorsiyon_Detaylı_Proje_Raporu.docx` / `.pdf`** — Projenin tam ve güncel raporu. Checkerboard tabanlı başlangıç kalibrasyonunu, yukarıda açıklanan gimbal + lazer tarama yöntemiyle birleştirerek kalibrasyon metodolojisinin tamamını uçtan uca belgeler.
- **`Checkerboard_Kalibrasyon_Sunum.pptx`** — Gimbal tabanlı sisteme geçilmeden önce, kameranın içsel parametrelerini belirlemek için kullanılan klasik satranç tahtası (Zhang yöntemi) kalibrasyonunu anlatan sunum. Yöntemin algoritması, alınan ölçümler ve elde edilen sonuçların karşılaştırmaları bu sunumda yer alır.
- **`Gimbal_Kalibrasyon_genel_perspektif_Sunum.pptx`** — Gimbal tabanlı distorsiyon ölçüm düzeneğinin genel hatlarını anlatan sunum: kullanılan cihazlar, alınan ölçümler ve karşılaşılan sorunlara yönelik öneriler bu sunumda özetlenmiştir.
- **`HeNe_lazer_sonrası_ve_karsilasilan_hatalar.pptx`** — Düzenekte fark edilen sistemsel hatanın olası kaynaklarını, kod ve donanım tarafında yapılabilecek değişiklikleri ve daha kararlı bir ölçüm için geçilen HeNe lazerini anlatan sunum. Hazırlayanlar: Ece Salman & Ekin Naz Partal.
- **`gimbal_system_visualization.py`** — Gimbal'in hareket aralığını 3 boyutlu olarak çizerek, fiziksel tarama sınırlarının görselleştirilmesini ve kafada canlandırılmasını kolaylaştıran bir prototip betik.

---

*Yüksek hassasiyetli Elektro-Optik Mühendisliği ve Ar-Ge uygulamaları için geliştirilmiştir.*

---

## English

# GZGSG Distortion Measurement & Camera Calibration

This repository contains an advanced **Opto-Mechanical Camera Calibration and Lens Distortion Measurement** system. It is designed to extract highly accurate intrinsic camera parameters ($f_x, f_y, c_x, c_y$) and distortion coefficients (Radial $k_1, k_2$ and Tangential $p_1, p_2$) using a laser-equipped gimbal system.

This project is particularly useful for high-precision optical systems such as **Star Trackers**, Computer Vision localization systems, and aerospace/defense imaging applications where eliminating optical barrel/pincushion distortion is mission-critical.

## 🚀 Features

- **Automated Gimbal Scanning:** Generates and executes a dense $NxN$ grid (e.g., 30x30) scanning path.
- **Anti-Backlash Navigation:** Uses a unidirectional (typewriter-style) scanning algorithm and dynamic dwell times to completely eliminate mechanical hysteresis and vibration (jitter).
- **Sub-Pixel Centroiding:** Implements an advanced Image Processing pipeline (Gaussian scattering suppression, Otsu thresholding, Intensity-Weighted Center of Mass) to locate the laser dot with sub-pixel accuracy.
- **Closed-Loop Synchronization:** Flawlessly syncs Python with Marlin Firmware (G-Code) using `M400` and `M114` to ensure images are captured only when physical movement has completely stopped.
- **Levenberg-Marquardt Optimization:** Solves the nonlinear physical angle-to-pixel projection model to extract the exact distortion array of the lens.
- **Automated Visualizations:** Generates composite mesh images overlaying the *ideal (undistorted)* grid with the *actual (measured)* grid to visually demonstrate distortion (e.g., Barrel/Pincushion).

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

The script feeds the measured laser pixel coordinates ($x_{pixel}, y_{pixel}$) into `scipy.optimize.least_squares` to reverse-engineer the variables ($f_x, f_y, c_x, c_y, k_1, k_2, p_1, p_2$).

## Usage

1. Mount the laser and camera on the gimbal and face a flat projection wall.
2. Update the `PORT` and `BAUD` settings in `gimbal_laser_calibration.py`.
3. Set your desired grid size (e.g., `GRID_SIZE_X = 30`, `GRID_SIZE_Y = 30`).
4. Run the script:
   ```bash
   python gimbal_laser_calibration.py
   ```
5. The script will automatically create a uniquely timestamped directory (e.g., `olcum_30x30_tarih_...`) containing:
   - Captured raw `.png` frames.
   - `sonuclar.csv` containing raw coordinate mappings.
   - `kalibrasyon_sonuc.csv` containing the final intrinsic matrix and distortion array.
   - `barrel_pincushion.png` visually showing the distortion curve.

## Notes on OpenCV Windows Bug

The script includes a custom `cv2_imwrite_utf8` wrapper function to bypass a known OpenCV C++ bug on Windows that prevents saving images to directories containing Unicode (Non-ASCII) characters.

## 📄 Reports & Documentation

- **`Gimbal_Distorsiyon_Detaylı_Proje_Raporu.docx` / `.pdf`** — The full, up-to-date project report. Documents the complete calibration methodology, combining the checkerboard baseline with the gimbal + laser scanning approach described above.
- **`Checkerboard_Kalibrasyon_Sunum.pptx`** — Presentation covering the classical checkerboard (Zhang's method) calibration used to determine the camera's intrinsic parameters before the gimbal-based system was introduced. Includes the algorithm explanation, measurements, and comparisons.
- **`Gimbal_Kalibrasyon_genel_perspektif_Sunum.pptx`** — General overview presentation of the gimbal-based distortion measurement setup: hardware used, measurements collected, and proposed solutions to encountered issues.
- **`HeNe_lazer_sonrası_ve_karsilasilan_hatalar.pptx`** — Discusses the likely causes of a systematic error identified in the setup, potential code- and hardware-level fixes, and the switch to a HeNe laser. Prepared by Ece Salman & Ekin Naz Partal.
- **`gimbal_system_visualization.py`** — A prototype script that plots the gimbal's 3D motion range, useful for visualizing the physical scanning envelope.

---

*Developed for high-precision Electro-Optical Engineering and R&D applications*
