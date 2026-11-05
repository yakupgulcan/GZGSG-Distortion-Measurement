import time
import csv
import os
import numpy as np
import cv2
from scipy.optimize import least_squares

import serial
from vmbpy import VmbSystem, PixelFormat, OPENCV_PIXEL_FORMATS
from datetime import datetime


PORT = 'COM10'
BAUD = 250000

# --- IZGARA VE MEKANİK AYARLAR ---
GRID_SIZE_X = 7      # X eksenindeki nokta sayısı (Azimut)
GRID_SIZE_Y = 7      # Y eksenindeki nokta sayısı (Elevasyon)

SAFE_LIMIT_X = 80.0       # Fiziksel limit 135.7 mm -> Güvenli sınır +-134 mm
SAFE_LIMIT_Y = 30.0       # Fiziksel limit 50.61 mm -> Güvenli sınır +-49 mm
STEP_RES = 0.02          # Step motorun çözünürlüğü (mm)

FEEDRATE = 300
HOMING_WAIT_SEC = 40
INITIAL_DWELL_SEC = 5.0    # Offset başlangıç noktasına gidince beklenecek süre
ROW_START_DWELL_SEC = 3.0  # Yeni satıra geçerken (uzun atlayış) bekleme süresi
DWELL_SEC = 0.7            # Normal ızgara adımlarında bekleme süresi

DEG_PER_MM_X = 1.0 / 5.0      # azimut (0.2 deg/mm)
DEG_PER_MM_Y = 1.0 / 2.37     # elevasyon (~0.422 deg/mm)

# --- FİZİKSEL ÇALIŞMA MESAFESİ VE OPTİK GEOMETRİ AYARLARI ---
# Pinhole çıkışı ile kamera lensinin ön yüzü arasındaki yaklaşık mesafe (~63 mm - 120 mm)
NOMINAL_DISTANCE_MM = 80.0    # Başlangıç tahmini (mm)

# --- AYDINLIK ORTAM VE AKILLI TAKİP (ROI) AYARLARI ---
SEARCH_RADIUS = 160           # Lazerin etrafındaki dinamik arama penceresi (piksel)
MIN_CONTRAST = 10.0           # Minimum yerel kontrast eşiği (DN)
OUTLIER_ERROR_THRESH_PX = 6.0 # Kalibrasyon fitinde aykırı sayılacak maksimum piksel hatası
# ---------------------------------------------------------------------

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
formatted_time_stamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

SAVE_DIR = os.path.join(BASE_DIR, f"olcum_{GRID_SIZE_X}x{GRID_SIZE_Y}_tarih_{formatted_time_stamp}")
print(f"[*] Kayıt Klasörü: {SAVE_DIR}")


# ---------------------------------------------------------------
# GIMBAL (Marlin / G-code) YARDIMCI FONKSİYONLARI
# ---------------------------------------------------------------
def send_and_wait(ser, cmd):
    ser.reset_input_buffer()
    ser.write((cmd + '\n').encode())
    ser.flush()
    
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line and line.startswith("X:"):
            print(f"  [M114 Gimbal Pos] {line}")
        if line.startswith("ok"):
            break


def wait_move_done(ser):
    send_and_wait(ser, 'M400')
    send_and_wait(ser, 'M114')


def round_stepper(val):
    return round(val / STEP_RES) * STEP_RES


def build_scan_plan(grid_x, grid_y, limit_x, limit_y):
    span_x = limit_x * 2.0
    span_y = limit_y * 2.0

    step_x = round_stepper(span_x / max(1, grid_x - 1))
    step_y = round_stepper(span_y / max(1, grid_y - 1))

    print(f"[*] Otomatik Step Boyutları: X = {step_x:.3f} mm, Y = {step_y:.3f} mm")

    half_x = round_stepper((grid_x - 1) * step_x / 2.0)
    half_y = round_stepper((grid_y - 1) * step_y / 2.0)

    x_positions = [round_stepper(-half_x + i * step_x) for i in range(grid_x)]
    y_positions = [round_stepper(+half_y - j * step_y) for j in range(grid_y)]

    plan = []
    current_x = x_positions[0]
    current_y = y_positions[0]
    point_no = 1

    for row in range(grid_y):
        y_target = y_positions[row]
        row_x = x_positions  # Tek yönlü (daktilo stili) tarama

        for step_idx, x_target in enumerate(row_x):
            dx = round_stepper(x_target - current_x)
            dy = round_stepper(y_target - current_y)
            actual_col = step_idx

            label = f"P{point_no:02d}_{x_target:.3f}_{y_target:.3f}"
            plan.append({
                "dx": dx, "dy": dy, 
                "label": label,
                "mm_x": x_target, "mm_y": y_target,
                "row": row, "col": actual_col
            })

            current_x = x_target
            current_y = y_target
            point_no += 1

    return plan, x_positions[0], y_positions[0]


# ---------------------------------------------------------------
# KAMERA (Vimba / vmbpy) - HIZLANDIRILMIŞ VE KİLİTLİ POZLAMA
# ---------------------------------------------------------------
class VimbaCamera:
    def __init__(self):
        self._vmb_cm = VmbSystem.get_instance()
        self._vmb_cm.__enter__()
        cams = self._vmb_cm.get_all_cameras()
        if not cams:
            raise RuntimeError("Vimba: hiçbir kamera bulunamadı.")
        self.cam = cams[0]
        self.cam.__enter__()

        supported_formats = self.cam.get_pixel_formats()
        if PixelFormat.Mono8 in supported_formats:
            self.cam.set_pixel_format(PixelFormat.Mono8)
        elif PixelFormat.Mono16 in supported_formats:
            self.cam.set_pixel_format(PixelFormat.Mono16)

        self._current_exposure_us = 15000.0
        self._apply_exposure(self._current_exposure_us)

    def _apply_exposure(self, exposure_us):
        exposure_us = float(np.clip(exposure_us, 50, 1_000_000))
        try:
            self.cam.ExposureTimeAbs.set(exposure_us)
        except Exception:
            try:
                self.cam.ExposureTime.set(exposure_us)
            except Exception:
                pass
        self._current_exposure_us = exposure_us

    def calibrate_exposure_once(self, target_peak=190):
        """
        Başlangıçta lazer pozlamasını 1 kez kalibre eder ve kilitler.
        Böylece tarama sırasında boş yere her adımda 6 kere beklemez,
        eskisi gibi ışık hızında (instant) çeker.
        """
        print("[*] Lazer başlangıç pozlaması optimize ediliyor...")
        for _ in range(5):
            img = self.capture_photo()
            peak = float(img.max())
            if peak >= 250:
                self._apply_exposure(self._current_exposure_us * 0.4)
                time.sleep(0.05)
            elif peak < 50:
                self._apply_exposure(self._current_exposure_us * 2.0)
                time.sleep(0.05)
            else:
                scale = target_peak / max(peak, 1.0)
                scale = float(np.clip(scale, 0.6, 1.5))
                self._apply_exposure(self._current_exposure_us * scale)
                break
        print(f"[*] Pozlama kilitlendi: {self._current_exposure_us:.0f} µs")

    def capture_photo(self):
        frame = self.cam.get_frame()
        if frame.get_pixel_format() not in OPENCV_PIXEL_FORMATS:
            src_fmt = frame.get_pixel_format()
            convertible_formats = src_fmt.get_convertible_formats()
            if PixelFormat.Mono8 in convertible_formats:
                frame = frame.convert_pixel_format(PixelFormat.Mono8)
            elif PixelFormat.Mono16 in convertible_formats:
                frame = frame.convert_pixel_format(PixelFormat.Mono16)
            else:
                raise RuntimeError("Kamera piksel formati donusturulemiyor.")

        img = frame.as_opencv_image()
        if img.ndim == 3:
            if img.shape[2] == 1:
                img = img[:, :, 0]
            elif img.shape[2] in (3, 4):
                img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return img

    def close(self):
        try:
            self.cam.__exit__(None, None, None)
        finally:
            self._vmb_cm.__exit__(None, None, None)


def cv2_imwrite_utf8(filename, img):
    ext = os.path.splitext(filename)[1]
    retval, buffer = cv2.imencode(ext, img)
    if retval:
        with open(filename, 'wb') as f:
            f.write(buffer)


# ---------------------------------------------------------------
# KUSURSUZ VE HIZLI DİNAMİK IZGARA TAKİBİ (ADAPTIVE TRACKER)
# ---------------------------------------------------------------
class AdaptiveGridTracker:
    """
    Kameranın masada durduğu gerçek pozisyona ve ızgaranın satır/sütun
    hareketlerine dinamik olarak uyum sağlar.
    
    1) Satır içi adımda bir önceki kolonun X koordinatından ilerler.
    2) Yeni satıra geçerken bir önceki satırın ortalama Y değerini baz alır;
       asla aşağıya, Thorlabs montaj halkasına kaymaz.
    3) Eğer arama kutusu bir anlığına kaçırırsa, tüm görüntüde lazeri otomatik
       bulup kutuyu anında lazerin üzerine kilitler (Global Fallback).
    """
    def __init__(self, search_radius=SEARCH_RADIUS, min_contrast=MIN_CONTRAST):
        self.search_radius = search_radius
        self.min_contrast = min_contrast

        # Gerçek laboratuvar ölçekleri (Bugünkü testlerden doğrulandı)
        self.kx = 4.1    # piksel / mm_x (yaklaşık 108 px / 26.66 mm)
        self.ky = 8.5    # piksel / mm_y (yaklaşık 85 px / 10.0 mm)

        self.detected_map = {}  # (row, col) -> (cx, cy)
        self.row_y_map = {}     # row -> ortalama y değeri
        self.initial_anchor_x = 670.0
        self.initial_anchor_y = 400.0
        self.has_first_anchor = False

    def predict(self, step):
        row = step["row"]
        col = step["col"]

        # 1. Strateji: Aynı satırda bir önceki sütun tespit edildiyse
        if (row, col - 1) in self.detected_map:
            prev_x, prev_y = self.detected_map[(row, col - 1)]
            dx = self.kx * step["dx"]
            return prev_x + dx, prev_y

        # 2. Strateji: Yeni satır başlangıcı (col == 0) -> Bir önceki satırın Y'sine göre in!
        # Böylece alt metal halkaya asla kaymaz!
        if (row - 1) in self.row_y_map:
            prev_row_y = self.row_y_map[row - 1]
            start_x = self.initial_anchor_x
            # Gimbal step["dy"] negatif olduğundan aşağı doğru inme:
            dy = -self.ky * step["dy"]
            return start_x, prev_row_y + dy

        # 3. Strateji: İlk nokta (P01)
        return self.initial_anchor_x, self.initial_anchor_y

    def detect_and_update(self, img, step):
        H, W = img.shape
        x_pred, y_pred = self.predict(step)
        r = self.search_radius

        x1 = max(0, int(round(x_pred - r)))
        x2 = min(W, int(round(x_pred + r + 1)))
        y1 = max(0, int(round(y_pred - r)))
        y2 = min(H, int(round(y_pred + r + 1)))

        roi = img[y1:y2, x1:x2]
        found = False
        cx, cy = float('nan'), float('nan')
        status_msg = ""

        # --- AŞAMA 1: YEREL ARAMA PENCERESİNDE HIZLI ARAMA ---
        if roi.size > 0:
            border = np.concatenate([roi[0, :], roi[-1, :], roi[:, 0], roi[:, -1]])
            bg = float(np.median(border))
            roi_sub = np.maximum(0.0, roi.astype(np.float32) - bg)
            roi_smooth = cv2.GaussianBlur(roi_sub, (5, 5), 0)
            _, max_v, _, max_loc = cv2.minMaxLoc(roi_smooth)

            if max_v >= self.min_contrast:
                thr = max(6.0, 0.35 * max_v)
                binary = (roi_sub >= thr).astype(np.uint8)
                n, labels, stats, _ = cv2.connectedComponentsWithStats(binary)
                if n > 1:
                    target_idx = labels[max_loc[1], max_loc[0]]
                    if target_idx == 0:
                        target_idx = 1 + stats[1:, cv2.CC_STAT_AREA].argmax()
                    area = stats[target_idx, cv2.CC_STAT_AREA]
                    if 2 <= area <= 6000:
                        mask = (labels == target_idx)
                        ys, xs = mask.nonzero()
                        w = roi_sub[ys, xs]
                        cx = float((xs * w).sum() / w.sum() + x1)
                        cy = float((ys * w).sum() / w.sum() + y1)
                        found = True
                        status_msg = f"OK (kontrast={max_v:.0f}, alan={area})"

        # --- AŞAMA 2: EĞER PENCEREDE YOKSA, KÜRESEL LAZER YAKALAYICI (FALLBACK) ---
        # Kutu yanlış bir yere baktıysa lazeri sensörde ara ve kutuyu anında lazerin üstüne çek!
        if not found:
            # 1 piksellik sıcak pikselleri (862, 138 gibi) ve dev oda ışıklarını ele
            thr_global = max(35, int(img.max() * 0.35))
            binary_g = (img >= thr_global).astype(np.uint8)
            n_g, labels_g, stats_g, centroids_g = cv2.connectedComponentsWithStats(binary_g)
            
            candidates = []
            for i in range(1, n_g):
                area_g = stats_g[i, cv2.CC_STAT_AREA]
                # Gerçek lazer beneği: 2 pikselden büyük, 10000 pikselden küçük (oda ışığı >50000'dir)
                if 2 <= area_g <= 10000:
                    candidates.append((area_g, centroids_g[i]))

            if candidates:
                # Varsa tahmine en yakın olanı, yoksa en belirgin olanı seç
                candidates.sort(key=lambda item: np.hypot(item[1][0] - x_pred, item[1][1] - y_pred))
                best_area, (gx, gy) = candidates[0]
                cx, cy = float(gx), float(gy)
                found = True
                status_msg = f"OK [KUTU HİZALANDI] ({cx:.1f}, {cy:.1f})"
                # Arama kutusunu doğrudan bulunan lazerin etrafına çek
                x1 = max(0, int(round(cx - r)))
                x2 = min(W, int(round(cx + r + 1)))
                y1 = max(0, int(round(cy - r)))
                y2 = min(H, int(round(cy + r + 1)))

        if found:
            self.detected_map[(step["row"], step["col"])] = (cx, cy)
            # Satır ortalama Y değerini güncelle
            row_pts = [pt[1] for k, pt in self.detected_map.items() if k[0] == step["row"]]
            self.row_y_map[step["row"]] = float(np.mean(row_pts))
            
            # İlk nokta (P01) ise başlangıç X çıpasını sabitle
            if not self.has_first_anchor and step["col"] == 0:
                self.initial_anchor_x = cx
                self.initial_anchor_y = cy
                self.has_first_anchor = True
                print(f"  [*] Başlangıç Çıpası Kilitlendi: ({cx:.1f}, {cy:.1f})")

            return cx, cy, (x1, y1, x2, y2), status_msg
        else:
            return float('nan'), float('nan'), (x1, y1, x2, y2), "NOT_DETECTED"


# ---------------------------------------------------------------
# TARAMA YÜRÜTME - MAKSİMUM HIZ & AKILLI GÖRSELLEŞTİRME
# ---------------------------------------------------------------
def run_scan(ser, camera, plan):
    results = []
    tracker = AdaptiveGridTracker()

    WINDOW_NAME = "Lazer Tarama - Hizli & Akilli Takip"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 950, 750)

    # 1. Başlangıçta pozlamayı tek seferde optimize et ve kilitle!
    camera.calibrate_exposure_once(target_peak=190)

    for step in plan:
        parts = []
        if step["dx"] != 0:
            parts.append(f"X{step['dx']:.3f}")
        if step["dy"] != 0:
            parts.append(f"Y{step['dy']:.3f}")

        if parts:
            send_and_wait(ser, "G1 " + " ".join(parts) + f" F{FEEDRATE}")
            wait_move_done(ser)
            wait_time = ROW_START_DWELL_SEC if step["col"] == 0 else DWELL_SEC
            print(f"[{step['label']}] Hedefe ulasildi, titresim icin {wait_time}sn bekleniyor...")
            time.sleep(wait_time)
        else:
            print(f"[{step['label']}] (Baslangic noktasindayiz, beklemeden foto cekiliyor...)")

        # 2. Hızlı çekim (Artık bekleme veya 6 kere yeniden deneme yok!)
        img = camera.capture_photo()
        fname = os.path.join(SAVE_DIR, f"{step['label']}.png")
        cv2_imwrite_utf8(fname, img)

        # 3. Akıllı tespit (Kutu asla kaymaz, lazeri takip eder)
        xu, yu, (rx1, ry1, rx2, ry2), status_msg = tracker.detect_and_update(img, step)

        if np.isnan(xu):
            print(f"  [!] NOT DETECTED: {step['label']}")
            pt_status = "NOT_DETECTED"
        else:
            print(f"  -> piksel = ({xu:.2f}, {yu:.2f}) [{status_msg}]")
            pt_status = "OK"

        # 4. Canlı görselleştirme
        display = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if img.ndim == 2 else img.copy()

        # Arama kutusu (Camgöbeği)
        cv2.rectangle(display, (rx1, ry1), (rx2, ry2), (255, 200, 0), 2)
        cv2.putText(display, "Takip Kutusu (ROI)", (rx1, max(20, ry1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 1)

        if pt_status == "OK":
            cv2.drawMarker(display, (int(round(xu)), int(round(yu))),
                           (0, 255, 0), markerType=cv2.MARKER_CROSS,
                           markerSize=25, thickness=2)
            cv2.putText(display, f"{step['label']}  px=({xu:.1f},{yu:.1f}) [OK]",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
        else:
            cv2.putText(display, f"{step['label']}  [NOT DETECTED - ATLANDI]",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

        cv2.imshow(WINDOW_NAME, display)
        cv2.waitKey(1)

        results.append({
            **step, "xu": xu, "yu": yu, "status": pt_status, "image_path": fname,
        })

    cv2.destroyWindow(WINDOW_NAME)
    return results


# ---------------------------------------------------------------
# FİT & GÖRSELLEŞTİRME (R & T Extrinsic + Brown-Conrady Modeli)
# ---------------------------------------------------------------
def project(params, theta_x, theta_y):
    """
    Gimbal açılarını (theta_x, theta_y radyan) 3B uzayda ışın/konum vektörüne çevirir,
    kameranın bacaklar üzerindeki açısal duruşu (R) ve mesafesi (T) ile dönüştürür,
    Brown-Conrady lens distorsiyonunu uygulayıp piksel koordinatlarına izdüşürür.
    
    params: [fx, fy, cx, cy, k1, k2, p1, p2, rx, ry, rz, tx, ty, tz]
    """
    fx, fy, cx, cy, k1, k2, p1, p2, rx, ry, rz, tx, ty, tz = params
    R, _ = cv2.Rodrigues(np.array([rx, ry, rz], dtype=np.float64))
    T = np.array([tx, ty, tz], dtype=np.float64)

    # 1. Pinhole 3B konumu (Gimbal referans koordinatlarında, mm cinsinden)
    # Z_gimbal mesafesi NOMINAL_DISTANCE_MM (~63-120 mm) baz alınır
    p_gimbal = NOMINAL_DISTANCE_MM * np.stack([np.tan(theta_x), np.tan(theta_y), np.ones_like(theta_x)], axis=-1)

    # 2. Kamera koordinat sistemine dönüşüm: P_cam = R @ P_gimbal + T (her iki taraf da mm)
    p_cam = (R @ p_gimbal.T).T + T

    # 3. Perspektif izdüşüm (Normalize koordinatlar: x' = X_c/Z_c, y' = Y_c/Z_c)
    Z_c = p_cam[:, 2]
    Z_safe = np.where(np.abs(Z_c) < 1e-6, 1e-6, Z_c)
    xprime = p_cam[:, 0] / Z_safe
    yprime = p_cam[:, 1] / Z_safe

    # 4. Lens distorsiyon modeli (Brown-Conrady)
    r2 = xprime**2 + yprime**2
    radial = 1.0 + k1 * r2 + k2 * r2**2
    x_dist = xprime * radial + 2.0 * p1 * xprime * yprime + p2 * (r2 + 2.0 * xprime**2)
    y_dist = yprime * radial + p1 * (r2 + 2.0 * yprime**2) + 2.0 * p2 * xprime * yprime

    # 5. İçsel matris (K) ile sensör pikseline izdüşüm
    xu = fx * x_dist + cx
    yu = fy * y_dist + cy
    return xu, yu


def fit_camera_params(results, fix_focal_length=False):
    valid = [r for r in results if not np.isnan(r["xu"])]
    if len(valid) < 14:
        raise RuntimeError(f"Fit için yeterli geçerli nokta yok ({len(valid)} nokta var, R ve T modeli için en az 14 gerekir).")

    print(f"\n[*] Toplam Ölçüm Noktası: {len(results)}")
    print(f"[*] Başarıyla Yakalanan Nokta: {len(valid)}")
    print(f"[*] Tespit Edilemeyen (Not Detected): {len(results) - len(valid)}")

    theta_x = np.radians(np.array([r["mm_x"] * DEG_PER_MM_X for r in valid]))
    theta_y = np.radians(np.array([r["mm_y"] * DEG_PER_MM_Y for r in valid]))

    xu_meas = np.array([r["xu"] for r in valid])
    yu_meas = np.array([r["yu"] for r in valid])

    def residuals(params, tx_rad, ty_rad, xm, ym):
        xu_pred, yu_pred = project(params, tx_rad, ty_rad)
        return np.concatenate([xm - xu_pred, ym - yu_pred])

    # Manta G-419 (2048x2048, 5.5 um) + Basler 12mm f/2.0 Lens:
    # Teorik odak uzaklığı: fx_teorik = 12 / 0.0055 ~ 2182 piksel.
    # [fx, fy, cx, cy, k1, k2, p1, p2, rx, ry, rz, tx, ty, tz]
    fx_init = 2252.0 if fix_focal_length else 2182.0
    fy_init = -2252.0 if fix_focal_length else -2182.0
    x0 = [
        fx_init, fy_init, 1024.0, 1024.0,   # Odak ve optik merkez
        0.0, 0.0, 0.0, 0.0,                # k1, k2, p1, p2
        0.0, 0.0, 0.0,                     # rx, ry, rz (Rotasyon)
        0.0, 0.0, 0.0                      # tx, ty, tz (mm sapma)
    ]

    # Fiziksel parametre sınırları
    fx_min = 2240.0 if fix_focal_length else 800.0
    fx_max = 2260.0 if fix_focal_length else 4000.0
    fy_min = -2260.0 if fix_focal_length else -4000.0
    fy_max = -2240.0 if fix_focal_length else -800.0
    
    lower = [
        fx_min, fy_min, 700.0, 700.0,
        -5.0, -10.0, -0.05, -0.05,
        -np.pi/4, -np.pi/4, -np.pi/4,
        -60.0, -60.0, -60.0
    ]
    upper = [
        fx_max, fy_max, 1350.0, 1350.0,
        5.0, 10.0, 0.05, 0.05,
        np.pi/4, np.pi/4, np.pi/4,
        60.0, 60.0, 60.0
    ]

    # 1. Aşama: Aykırı noktalara (outlier) karşı dirençli Soft-L1 optimizasyonu (TRF)
    res_robust = least_squares(
        lambda p: residuals(p, theta_x, theta_y, xu_meas, yu_meas),
        x0, bounds=(lower, upper), loss='soft_l1', f_scale=2.0, max_nfev=30000
    )

    xu_pred, yu_pred = project(res_robust.x, theta_x, theta_y)
    errors = np.hypot(xu_meas - xu_pred, yu_meas - yu_pred)
    med_err = float(np.median(errors))

    outlier_thresh = max(OUTLIER_ERROR_THRESH_PX, 3.5 * med_err)
    inliers = errors <= outlier_thresh
    num_outliers = int((~inliers).sum())

    if num_outliers > 0 and inliers.sum() >= 14:
        print(f"\n[!] GÜVENLİK FİLTRESİ: {num_outliers} adet hatalı nokta tespit edildi ve elendi!")
        for idx in np.where(~inliers)[0]:
            print(f"    - Elenen Nokta: {valid[idx]['label']} (Sapma: {errors[idx]:.2f} px > Eşik: {outlier_thresh:.2f} px)")
            valid[idx]["status"] = "OUTLIER_EXCLUDED"

        tx_c = theta_x[inliers]
        ty_c = theta_y[inliers]
        xu_c = xu_meas[inliers]
        yu_c = yu_meas[inliers]

        final_res = least_squares(
            lambda p: residuals(p, tx_c, ty_c, xu_c, yu_c),
            res_robust.x, bounds=(lower, upper), method='trf', max_nfev=30000
        )
        final_params = final_res.x
        final_rms = float(np.sqrt(np.mean(final_res.fun**2)))
    else:
        final_params = res_robust.x
        final_rms = float(np.sqrt(np.mean(res_robust.fun**2)))

    fx, fy, cx, cy, k1, k2, p1, p2, rx, ry, rz, tx, ty, tz = final_params
    rx_deg, ry_deg, rz_deg = np.degrees([rx, ry, rz])

    return {
        "fx": fx, "fy": fy, "cx": cx, "cy": cy,
        "k1": k1, "k2": k2, "p1": p1, "p2": p2,
        "rx_rad": rx, "ry_rad": ry, "rz_rad": rz,
        "pitch_deg": rx_deg, "yaw_deg": ry_deg, "roll_deg": rz_deg,
        "tx_mm": tx, "ty_mm": ty, "tz_mm": tz,
        "rms_px": final_rms,
        "valid_points": int(inliers.sum()) if num_outliers > 0 else len(valid),
        "outliers_removed": num_outliers,
        "full_params": list(final_params)
    }


def save_composite(results, img_shape, out_path):
    composite = np.zeros(img_shape, dtype=np.uint8)
    composite_color = cv2.cvtColor(composite, cv2.COLOR_GRAY2BGR)
    point_map = {(r["row"], r["col"]): r for r in results}

    for r in results:
        if np.isnan(r["xu"]): continue
        pt = (int(round(r["xu"])), int(round(r["yu"])))

        for d_row, d_col in [(0, 1), (1, 0)]:
            neighbor = point_map.get((r["row"] + d_row, r["col"] + d_col))
            if neighbor is not None and not np.isnan(neighbor["xu"]):
                pt2 = (int(round(neighbor["xu"])), int(round(neighbor["yu"])))
                cv2.line(composite_color, pt, pt2, (0, 255, 255), 1)

        cv2.drawMarker(composite_color, pt, (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=14, thickness=2)

    cv2_imwrite_utf8(out_path, composite_color)


def save_distortion_comparison(results, fit_params, img_shape, out_path):
    composite = np.zeros(img_shape, dtype=np.uint8)
    composite_color = cv2.cvtColor(composite, cv2.COLOR_GRAY2BGR)
    point_map = {(r["row"], r["col"]): r for r in results}

    full_p = fit_params.get("full_params")
    if full_p is not None:
        # İdeal referans ızgara: Distorsiyonsuz (k1=0, k2=0, p1=0, p2=0), ancak R ve T geometrisi dahil
        ideal_params = list(full_p)
        ideal_params[4] = 0.0  # k1 = 0
        ideal_params[5] = 0.0  # k2 = 0
        ideal_params[6] = 0.0  # p1 = 0
        ideal_params[7] = 0.0  # p2 = 0

        valid_results = [r for r in results]
        tx_all = np.radians(np.array([r["mm_x"] * DEG_PER_MM_X for r in valid_results]))
        ty_all = np.radians(np.array([r["mm_y"] * DEG_PER_MM_Y for r in valid_results]))
        ix_all, iy_all = project(ideal_params, tx_all, ty_all)

        for i, r in enumerate(valid_results):
            r["ideal_pt"] = (int(round(ix_all[i])), int(round(iy_all[i])))
    else:
        for r in results:
            r["ideal_pt"] = (int(round(r.get("xu", 0))), int(round(r.get("yu", 0))))

    # İdeal ızgara (Yeşil)
    for r in results:
        pt = r["ideal_pt"]
        for d_row, d_col in [(0, 1), (1, 0)]:
            neighbor = point_map.get((r["row"] + d_row, r["col"] + d_col))
            if neighbor is not None and "ideal_pt" in neighbor:
                cv2.line(composite_color, pt, neighbor["ideal_pt"], (0, 255, 0), 1)

    # Gerçek ölçülen ızgara (Sarı çizgiler, kırmızı artılar)
    for r in results:
        if np.isnan(r["xu"]): continue
        pt = (int(round(r["xu"])), int(round(r["yu"])))
        for d_row, d_col in [(0, 1), (1, 0)]:
            neighbor = point_map.get((r["row"] + d_row, r["col"] + d_col))
            if neighbor is not None and not np.isnan(neighbor["xu"]):
                pt2 = (int(round(neighbor["xu"])), int(round(neighbor["yu"])))
                cv2.line(composite_color, pt, pt2, (0, 255, 255), 1)
        cv2.drawMarker(composite_color, pt, (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=1)

    cv2.putText(composite_color, "Yesil=ideal (distorsiyonsuz R,T)  Sari=olcum (gercek)",
                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    cv2_imwrite_utf8(out_path, composite_color)


def save_csv(results, out_path):
    fieldnames = ["label", "mm_x", "mm_y", "row", "col", "xu", "yu", "status", "image_path"]
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in results:
            writer.writerow({k: r.get(k, "") for k in fieldnames})


def calibrate_from_csv(csv_path, fix_focal_length=False):
    """
    Daha önce kaydedilmiş bir sonuclar.csv dosyasından kalibrasyon fitini
    donanıma bağlanmaya gerek kalmadan tekrar çalıştırır ve raporlar.
    """
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = list(csv.DictReader(f))
    
    results = []
    for r in reader:
        xu_val = float(r['xu']) if r['xu'] and r['xu'] != 'nan' else float('nan')
        yu_val = float(r['yu']) if r['yu'] and r['yu'] != 'nan' else float('nan')
        results.append({
            'label': r.get('label', ''),
            'mm_x': float(r['mm_x']),
            'mm_y': float(r['mm_y']),
            'row': int(r.get('row', 0)),
            'col': int(r.get('col', 0)),
            'xu': xu_val,
            'yu': yu_val,
            'status': r.get('status', 'OK')
        })

    fit_params = fit_camera_params(results, fix_focal_length=fix_focal_length)
    opt_dist_pct = (fit_params['k1'] * 0.412 + fit_params['k2'] * (0.412**2)) * 100
    
    print("\n=== ÇEVRİMDIŞI KALİBRASYON SONUCU ===")
    print(f"fx = {fit_params['fx']:.2f} px | fy = {fit_params['fy']:.2f} px  (Basler 12mm nominal: 2252 px)")
    print(f"cx = {fit_params['cx']:.2f} px | cy = {fit_params['cy']:.2f} px  (Sensör merkezi: 1024, 1024)")
    print(f"Boresight Açıları: Pitch={fit_params['pitch_deg']:.2f}° | Yaw={fit_params['yaw_deg']:.2f}° | Roll={fit_params['roll_deg']:.2f}°")
    print(f"Pinhole Konum Sapması: Tx={fit_params['tx_mm']:.2f} mm | Ty={fit_params['ty_mm']:.2f} mm | Tz={fit_params['tz_mm']:.2f} mm")
    print(f"Radyal Distorsiyon: k1 = {fit_params['k1']:.6f} | k2 = {fit_params['k2']:.6f}")
    print(f"Tanjansiyel Distorsiyon: p1 = {fit_params['p1']:.6f} | p2 = {fit_params['p2']:.6f}")
    print(f"RMS Hatası = {fit_params['rms_px']:.4f} piksel")
    print(f"Hesaplanan Köşe Optik Distorsiyonu (%) = {opt_dist_pct:.2f}%  (Basler Katalog Hedefi: -3.10% | k1 ~ -0.075)")
    return fit_params


# ---------------------------------------------------------------
# ANA PROGRAM
# ---------------------------------------------------------------
def main():
    os.makedirs(SAVE_DIR, exist_ok=True)
    ser = serial.Serial(PORT, BAUD, timeout=1)
    camera = VimbaCamera()

    ser.reset_input_buffer()
    
    print(f"Homing bekleniyor ({HOMING_WAIT_SEC} sn)...")
    time.sleep(HOMING_WAIT_SEC)
    
    send_and_wait(ser, 'G92 X0 Y0')
    send_and_wait(ser, 'G91')

    plan, start_x, start_y = build_scan_plan(GRID_SIZE_X, GRID_SIZE_Y, SAFE_LIMIT_X, SAFE_LIMIT_Y)
    
    print(f"\n=== OFFSET: İlk başlangıç noktasına gidiliyor (X={start_x:.3f}, Y={start_y:.3f}) ===")
    send_and_wait(ser, f"G1 X{start_x:.3f} Y{start_y:.3f} F{FEEDRATE}")
    wait_move_done(ser)
    print(f"Offset'e ulaşıldı. Titreşimin sönmesi için {INITIAL_DWELL_SEC} sn bekleniyor...")
    time.sleep(INITIAL_DWELL_SEC)

    print(f"\n=== {GRID_SIZE_X}x{GRID_SIZE_Y} Tarama Başlıyor ({len(plan)} nokta) ===\n")
    results = run_scan(ser, camera, plan)

    print(f"\n=== Tarama tamamlandı, {len(results)} nokta işlendi ===\n")
    save_csv(results, os.path.join(SAVE_DIR, "sonuclar.csv"))

    sample_img = camera.capture_photo()
    img_shape = sample_img.shape

    save_composite(results, img_shape, os.path.join(SAVE_DIR, "composite_mesh.png"))

    try:
        fit_params = fit_camera_params(results)
        print("\n=== KALİBRASYON SONUCU (R & T Extrinsic + Lens Distorsiyonu) ===")
        print(f"fx = {fit_params['fx']:.2f} px | fy = {fit_params['fy']:.2f} px  (Manta G-419 + Basler 12mm teorik: ~2182 px)")
        print(f"cx = {fit_params['cx']:.2f} px | cy = {fit_params['cy']:.2f} px  (Sensör merkezi: 1024, 1024)")
        print(f"Rotasyon (Boresight Eğim Açısı): Pitch={fit_params['pitch_deg']:.2f}° | Yaw={fit_params['yaw_deg']:.2f}° | Roll={fit_params['roll_deg']:.2f}°")
        print(f"Öteleme (Pinhole-Kamera Konumu): Tx={fit_params['tx_mm']:.1f} mm | Ty={fit_params['ty_mm']:.1f} mm | Tz={fit_params['tz_mm']:.1f} mm")
        print(f"Radyal distorsiyon: k1 = {fit_params['k1']:.6f} | k2 = {fit_params['k2']:.6f}")
        print(f"Tanjansiyel distorsiyon: p1 = {fit_params['p1']:.6f} | p2 = {fit_params['p2']:.6f}")
        print(f"RMS Hatası = {fit_params['rms_px']:.4f} piksel")
        print(f"Kullanılan Temiz Nokta Sayısı: {fit_params['valid_points']}")

        if fit_params["k1"] < 0:
            print("Yorum: k1 < 0 -> BARREL (fıçı) distorsiyonu baskın.")
        elif fit_params["k1"] > 0:
            print("Yorum: k1 > 0 -> PINCUSHION (yastık) distorsiyonu baskın.")

        save_distortion_comparison(results, fit_params, img_shape,
                                    os.path.join(SAVE_DIR, "barrel_pincushion.png"))

        with open(os.path.join(SAVE_DIR, "kalibrasyon_sonuc.csv"), "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            for k, v in fit_params.items():
                if k != "full_params":
                    writer.writerow([k, v])

    except RuntimeError as e:
        print(f"\nFit yapılamadı: {e}")

    send_and_wait(ser, 'M17')
    send_and_wait(ser, 'M84 S0')

    print("\nGimbal son konumda bekliyor. Kapatmak için CTRL+C.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Kapatılıyor...")
    finally:
        camera.close()
        ser.close()


if __name__ == "__main__":
    main()
