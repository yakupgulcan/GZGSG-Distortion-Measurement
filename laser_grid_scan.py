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

# --- YENİ EKLENEN/GÜNCELLENEN AYARLAR ---
GRID_SIZE_X = 15      # X eksenindeki nokta sayısı (Azimut)
GRID_SIZE_Y = 15      # Y eksenindeki nokta sayısı (Elevasyon)

SAFE_LIMIT_X = 80.0      # Fiziksel limit 135.7 mm -> Güvenli sınır +-134 mm
SAFE_LIMIT_Y = 30.0       # Fiziksel limit 50.61 mm -> Güvenli sınır +-49 mm
STEP_RES = 0.02         # Step motorun çözünürlüğü (mm) azimut=0.004mm VE elevasyon=0.005mm dolayısıyla bunların ekoku olan 0.02 seçildi

FEEDRATE = 300
HOMING_WAIT_SEC = 40
INITIAL_DWELL_SEC = 5.0   # Offset başlangıç noktasına gidince beklenecek süre
ROW_START_DWELL_SEC = 3.0 # Yeni satıra geçerken (uzun atlayış) sarsıntıyı önleme süresi
DWELL_SEC = 0.7           # Normal ızgara adımlarında (kısa adım) bekleme süresi
# ----------------------------------------

DEG_PER_MM_X = 1.0 / 5.0     # azimut
DEG_PER_MM_Y = 1.0 / 2.37    # elevasyon

# Dosyanın bulunduğu klasörü mutlak olarak bul (Nerede çalıştırılırsa çalıştırılsın klasör doğru yere açılır)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
formatted_time_stamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

# Her yeni ölçümü "olcum_30x30_tarih_Y-A-G_S-D-S" şeklinde yeni bir klasöre kaydet
SAVE_DIR = os.path.join(BASE_DIR, f"olcum_{GRID_SIZE_X}x{GRID_SIZE_Y}_tarih_{formatted_time_stamp}")
print(f"[*] Kayıt Klasörü: {SAVE_DIR}")


# ---------------------------------------------------------------
# GIMBAL (Marlin / G-code) YARDIMCI FONKSİYONLARI
# ---------------------------------------------------------------
def send_and_wait(ser, cmd):
    """
    Komutu göndermeden önce seri port tamponunu (buffer) temizler,
    böylece eski/bayat 'ok' mesajlarını okumayı engeller. Gerçek 'ok' 
    gelene kadar Python'u kesin olarak bloklar.
    """
    ser.reset_input_buffer()
    ser.write((cmd + '\n').encode())
    ser.flush()
    
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        
        # Gelen yanıt boş değilse ve M114 posizyon verisi içeriyorsa ekrana bas
        if line and line.startswith("X:"):
            print(f"  [M114 Gimbal Pos] {line}")
            
        if line.startswith("ok"):
            break


def wait_move_done(ser):
    """
    M400 ile tüm hareketlerin bitmesini bekler.
    Ardından M114 ile konumu sorgulayarak ekstra doğrulama ve senkronizasyon sağlar.
    """
    send_and_wait(ser, 'M400')
    send_and_wait(ser, 'M114')


def round_stepper(val):
    """Değeri step motorun çözünürlüğüne (0.004 mm) yuvarlar."""
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
    
    # DİKKAT: Tarama döngüsü artık offset noktasından (-half_x, +half_y) başlayacak.
    # Bu yüzden current_x ve current_y 0 değil, ilk hedefin koordinatları olarak başlıyor.
    current_x = x_positions[0]
    current_y = y_positions[0]
    point_no = 1

    for row in range(grid_y):
        y_target = y_positions[row]
        
        # Zikzak (backlash) hatasını önlemek için Yılan Kavi iptal edildi!
        # Daktilo gibi hep soldan sağa (tek yönlü) tarama yapılacak.
        row_x = x_positions

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

    # Başlangıç offset koordinatlarını da main() fonksiyonuna döndürüyoruz
    return plan, x_positions[0], y_positions[0]


# ---------------------------------------------------------------
# KAMERA (Vimba / vmbpy)
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

        self._current_exposure_us = 20000.0
        try:
            self.cam.ExposureTimeAbs.set(self._current_exposure_us)
        except Exception:
            try:
                self.cam.ExposureTime.set(self._current_exposure_us)
            except Exception:
                pass

    def _set_exposure(self, exposure_us):
        exposure_us = float(np.clip(exposure_us, 20, 1_000_000))
        try:
            self.cam.ExposureTimeAbs.set(exposure_us)
        except Exception:
            try:
                self.cam.ExposureTime.set(exposure_us)
            except Exception:
                pass
        self._current_exposure_us = exposure_us

    def capture_photo_auto_exposure(self, target_max=200, max_tries=6,
                                     saturation_limit=250, min_signal=30):
        img = None
        for attempt in range(max_tries):
            img = self.capture_photo()
            peak = float(img.max())

            if peak >= saturation_limit:
                self._set_exposure(self._current_exposure_us * 0.4)
                continue
            elif peak < min_signal:
                self._set_exposure(self._current_exposure_us * 2.5)
                continue
            else:
                scale = target_max / max(peak, 1.0)
                scale = float(np.clip(scale, 0.5, 2.0))
                self._set_exposure(self._current_exposure_us * scale)
                return img
        return img

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
                raise RuntimeError(f"Kamera piksel formati donusturulemiyor.")

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
    """
    OpenCV'nin (cv2.imwrite) Windows'ta Türkçe karakterli ('İ', 'Ş' vb.) klasör
    yollarına fotoğraf kaydedememe (çökme veya sessizce başarısız olma) bug'ını 
    aşmak için yazılmış özel kayıt fonksiyonudur.
    """
    ext = os.path.splitext(filename)[1]
    retval, buffer = cv2.imencode(ext, img)
    if retval:
        with open(filename, 'wb') as f:
            f.write(buffer)


# ---------------------------------------------------------------
# LAZER MERKEZİ TESPİTİ
# ---------------------------------------------------------------
def find_laser_centroid(img):
    """
    Güçlü Gaussian Blur ile saçılmalara (speckle/diffraction) karşı dirençli
    ROI + connected components + intensity weighted centroid algoritması.
    """
    img_f = img.astype(np.float32)

    # 1. ADIM: Saçılmaları birleştirmek için geçici olarak yoğun bir Blur uygula
    # Bu blur sadece "kabaca en yoğun bölgeyi (dağın zirvesini)" bulmak içindir.
    blurred_for_max = cv2.GaussianBlur(img_f, (41, 41), 0)
    
    # En parlak noktayı şimdi bu bulanıklaştırılmış görüntüde arıyoruz
    _, max_val, _, max_loc = cv2.minMaxLoc(blurred_for_max)
    if max_val <= 0:
        raise RuntimeError("Lazer noktasi bulunamadi.")

    # 2. ADIM: Saçılmaları da kapsayabilmesi için ROI boyutunu büyütüyoruz (örn: 60)
    roi_size = 60 
    x0, y0 = max_loc
    x1 = max(0, x0 - roi_size)
    x2 = min(img.shape[1], x0 + roi_size + 1)
    y1 = max(0, y0 - roi_size)
    y2 = min(img.shape[0], y0 + roi_size + 1)

    roi = img_f[y1:y2, x1:x2]
    
    # ROI üzerinde hesaplama yapmadan önce hafif bir gürültü temizleme
    roi = cv2.GaussianBlur(roi, (5, 5), 0)

    # 3. ADIM: Eşikleme (Threshold). Çok düşük yaparsak saçılmalar hesabı bozar, 
    # 0.5 ile 0.7 arası idealdir.
    thr = 0.5 * roi.max()
    binary = (roi >= thr).astype("uint8")

    # 4. ADIM: Connected Components
    n, labels, stats, _ = cv2.connectedComponentsWithStats(binary)
    if n <= 1:
        raise RuntimeError("Lazer blob'u bulunamadi.")

    # En büyük alanı (arka plan hariç) seç
    idx = 1 + stats[1:, cv2.CC_STAT_AREA].argmax()
    mask = labels == idx

    # 5. ADIM: Yoğunluk Ağırlıklı Merkez (Intensity Weighted Centroid) Hesabı
    ys, xs = mask.nonzero()
    w = roi[ys, xs]
    weight_sum = w.sum()
    
    if weight_sum <= 0:
        raise RuntimeError("Lazer centroid hesabi yapilamadi.")

    cx = (xs * w).sum() / weight_sum + x1
    cy = (ys * w).sum() / weight_sum + y1
    
    return float(cx), float(cy)


# ---------------------------------------------------------------
# TARAMA YÜRÜTME
# ---------------------------------------------------------------
def run_scan(ser, camera, plan):
    results = []
    WINDOW_NAME = "Lazer Tarama - Canli Goruntu"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 900, 700)

    for step in plan:
        parts = []
        if step["dx"] != 0:
            parts.append(f"X{step['dx']:.3f}")
        if step["dy"] != 0:
            parts.append(f"Y{step['dy']:.3f}")

        # Eğer hareket varsa git ve bekle. İlk adımda (P01) dx=0 ve dy=0 olacağı için
        # motorlara tekrar komut gönderilmez, doğrudan foto çeker.
        if parts:
            send_and_wait(ser, "G1 " + " ".join(parts) + f" F{FEEDRATE}")
            wait_move_done(ser)
            
            # Eğer satırın ilk noktasıysa (uzun atlayış), ROW_START_DWELL_SEC kadar bekle
            wait_time = ROW_START_DWELL_SEC if step["col"] == 0 else DWELL_SEC
            
            print(f"[{step['label']}] Hedefe ulasildi, titresim icin {wait_time}sn bekleniyor...")
            time.sleep(wait_time)
        else:
            print(f"[{step['label']}] (Baslangic noktasindayiz, beklemeden foto cekiliyor...)")

        img = camera.capture_photo_auto_exposure()
        fname = os.path.join(SAVE_DIR, f"{step['label']}.png")
        cv2_imwrite_utf8(fname, img)

        try:
            xu, yu = find_laser_centroid(img)
        except RuntimeError as e:
            print(f"  UYARI: {step['label']} -> {e}")
            xu, yu = float('nan'), float('nan')

        print(f"  -> piksel = ({xu:.2f}, {yu:.2f})")

        # Canlı görüntü
        display = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if img.ndim == 2 else img.copy()
        if not np.isnan(xu):
            cv2.drawMarker(display, (int(round(xu)), int(round(yu))),
                            (0, 0, 255), markerType=cv2.MARKER_CROSS,
                            markerSize=25, thickness=2)
        cv2.putText(display, f"{step['label']}  px=({xu:.1f},{yu:.1f})",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow(WINDOW_NAME, display)
        cv2.waitKey(1)

        results.append({
            **step, "xu": xu, "yu": yu, "image_path": fname,
        })

    cv2.destroyWindow(WINDOW_NAME)
    return results


# ---------------------------------------------------------------
# FİT & GÖRSELLEŞTİRME (Aynı)
# ---------------------------------------------------------------
def project(params, xprime, yprime):
    fx, fy, cx, cy, k1, k2, p1, p2 = params
    r2 = xprime**2 + yprime**2
    x_dist = xprime * (1 + k1*r2 + k2*r2**2) + 2*p1*xprime*yprime + p2*(r2 + 2*xprime**2)
    y_dist = yprime * (1 + k1*r2 + k2*r2**2) + p1*(r2 + 2*yprime**2) + 2*p2*xprime*yprime
    xu = fx * x_dist + cx
    yu = fy * y_dist + cy
    return xu, yu

def fit_camera_params(results):
    valid = [r for r in results if not np.isnan(r["xu"])]
    if len(valid) < 8:
        raise RuntimeError("Fit için yeterli geçerli nokta yok (en az 8 gerekir).")

    theta_x = np.array([r["mm_x"] * DEG_PER_MM_X for r in valid])
    theta_y = np.array([r["mm_y"] * DEG_PER_MM_Y for r in valid])
    xprime = np.tan(np.radians(theta_x))
    yprime = np.tan(np.radians(theta_y))

    xu_meas = np.array([r["xu"] for r in valid])
    yu_meas = np.array([r["yu"] for r in valid])

    def residuals(params):
        xu_pred, yu_pred = project(params, xprime, yprime)
        return np.concatenate([xu_meas - xu_pred, yu_meas - yu_pred])

    x0 = [2290, 2290, 1024, 1024, 0, 0, 0, 0]
    result = least_squares(residuals, x0, method='lm', max_nfev=20000)
    fx, fy, cx, cy, k1, k2, p1, p2 = result.x
    rms = np.sqrt(np.mean(result.fun**2))
    
    return {
        "fx": fx, "fy": fy, "cx": cx, "cy": cy,
        "k1": k1, "k2": k2, "p1": p1, "p2": p2, "rms_px": rms,
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
    fx, fy, cx, cy = fit_params["fx"], fit_params["fy"], fit_params["cx"], fit_params["cy"]
    point_map = {(r["row"], r["col"]): r for r in results}

    for r in results:
        xp = np.tan(np.radians(r["mm_x"] * DEG_PER_MM_X))
        yp = np.tan(np.radians(r["mm_y"] * DEG_PER_MM_Y))
        r["ideal_pt"] = (int(round(fx * xp + cx)), int(round(fy * yp + cy)))

    for r in results:
        pt = r["ideal_pt"]
        for d_row, d_col in [(0, 1), (1, 0)]:
            neighbor = point_map.get((r["row"] + d_row, r["col"] + d_col))
            if neighbor is not None:
                cv2.line(composite_color, pt, neighbor["ideal_pt"], (0, 255, 0), 1)

    for r in results:
        if np.isnan(r["xu"]): continue
        pt = (int(round(r["xu"])), int(round(r["yu"])))
        for d_row, d_col in [(0, 1), (1, 0)]:
            neighbor = point_map.get((r["row"] + d_row, r["col"] + d_col))
            if neighbor is not None and not np.isnan(neighbor["xu"]):
                pt2 = (int(round(neighbor["xu"])), int(round(neighbor["yu"])))
                cv2.line(composite_color, pt, pt2, (0, 255, 255), 1)
        cv2.drawMarker(composite_color, pt, (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=1)

    cv2.putText(composite_color, "Yesil=ideal (distorsiyonsuz)  Sari=olcum (gercek)",
                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    cv2_imwrite_utf8(out_path, composite_color)

def save_csv(results, out_path):
    fieldnames = ["label", "mm_x", "mm_y", "row", "col", "xu", "yu", "image_path"]
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in results:
            writer.writerow({k: r[k] for k in fieldnames})


# ---------------------------------------------------------------
# ANA PROGRAM
# ---------------------------------------------------------------
def main():
    os.makedirs(SAVE_DIR, exist_ok=True)
    ser = serial.Serial(PORT, BAUD, timeout=1)
    camera = VimbaCamera()

    # Portu ilk açtığımızda tamponu temizleyelim
    ser.reset_input_buffer()
    
    print(f"Homing bekleniyor ({HOMING_WAIT_SEC} sn)...")
    time.sleep(HOMING_WAIT_SEC)
    
    # 1. Homing sonrası mevcut konumu G92 ile mutlak 0,0 yap
    send_and_wait(ser, 'G92 X0 Y0')
    
    # 2. Göreli (Relative) Koordinat moduna geçir
    send_and_wait(ser, 'G91')

    # Planı ve offset koordinatlarını oluştur
    plan, start_x, start_y = build_scan_plan(GRID_SIZE_X, GRID_SIZE_Y, SAFE_LIMIT_X, SAFE_LIMIT_Y)
    
    print(f"\n=== OFFSET: ece İlk baslangic noktasina gidiliyor (X={start_x:.3f}, Y={start_y:.3f}) ===")
    send_and_wait(ser, f"G1 X{start_x:.3f} Y{start_y:.3f} F{FEEDRATE}")
    wait_move_done(ser)
    print(f"Offset'e ulaşıldı. Titreşimin sönmesi için {INITIAL_DWELL_SEC} sn bekleniyor...")
    time.sleep(INITIAL_DWELL_SEC)

    print(f"\n=== {GRID_SIZE_X}x{GRID_SIZE_Y} tarama başlıyor ({len(plan)} nokta) ===\n")
    results = run_scan(ser, camera, plan)

    print(f"\n=== Tarama tamamlandı, {len(results)} nokta işlendi ===\n")
    save_csv(results, os.path.join(SAVE_DIR, "sonuclar.csv"))

    sample_img = camera.capture_photo()
    img_shape = sample_img.shape

    save_composite(results, img_shape, os.path.join(SAVE_DIR, "composite_mesh.png"))

    try:
        fit_params = fit_camera_params(results)
        print("\n=== KALİBRASYON SONUCU ===")
        print(f"fx = {fit_params['fx']:.3f} | fy = {fit_params['fy']:.3f}")
        print(f"cx = {fit_params['cx']:.3f} | cy = {fit_params['cy']:.3f}")
        print(f"k1 = {fit_params['k1']:.6f} | k2 = {fit_params['k2']:.6f}")
        print(f"p1 = {fit_params['p1']:.6f} | p2 = {fit_params['p2']:.6f}")
        print(f"RMS = {fit_params['rms_px']:.4f} piksel")

        if fit_params["k1"] < 0:
            print("Yorum: k1 < 0 -> BARREL (fıçı) distorsiyonu baskın.")
        elif fit_params["k1"] > 0:
            print("Yorum: k1 > 0 -> PINCUSHION (yastık) distorsiyonu baskın.")

        save_distortion_comparison(results, fit_params, img_shape,
                                    os.path.join(SAVE_DIR, "barrel_pincushion.png"))

        with open(os.path.join(SAVE_DIR, "kalibrasyon_sonuc.csv"), "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            for k, v in fit_params.items():
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