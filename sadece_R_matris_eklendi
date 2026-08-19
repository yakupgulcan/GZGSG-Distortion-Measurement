import time
import csv
import os
import numpy as np
import cv2
from scipy.optimize import least_squares

import serial
from vmbpy import VmbSystem, PixelFormat, OPENCV_PIXEL_FORMATS
from datetime import datetime


# ===============================================================
# GIMBAL + KAMERA KALIBRASYON SISTEMI
#
# MODEL:
#
#       GIMBAL KOMUTU
#             |
#             v
#       alpha, beta
#             |
#             v
#   COUPLED BEAM MODEL
#       x' = tan(alpha)
#       y' = -tan(beta) / cos(alpha)
#             |
#             v
#       GLOBAL BORESIGHT R
#             |
#             v
#      CAMERA NORMALIZED
#        COORDINATES
#             |
#             v
#     DISTORTION MODEL
#             |
#             v
#        PIXEL (u,v)
#
#
# ONEMLI:
#   Bu modelde T YOKTUR.
#
#   R sadece bir tane global sabit donusumdur.
#   Butun tarama boyunca ayni R kullanilir.
#
# ===============================================================


# ===============================================================
# SERVO / GIMBAL AYARLARI
# ===============================================================

PORT = 'COM10'
BAUD = 250000


# ===============================================================
# TARAMA AYARLARI
# ===============================================================

GRID_SIZE_X = 30
GRID_SIZE_Y = 30

# ---------------------------------------------------------------
# GUVENLI TARAMA SINIRLARI
#
# Simetrik hale getirildi.
#
# Son kullandigimiz guvenli ortak sinir:
#       X = +/- 50.0 mm
#       Y = +/- 23.7 mm
#
# Eger home-bazli son mekanik olcumleriniz bundan farkliysa
# SADECE bu iki degeri degistirmeniz yeterli.
# ---------------------------------------------------------------

SAFE_LIMIT_X = 50.0
SAFE_LIMIT_Y = 23.7

STEP_RES = 0.02

FEEDRATE = 300

HOMING_WAIT_SEC = 40
INITIAL_DWELL_SEC = 5.0
ROW_START_DWELL_SEC = 3.0
DWELL_SEC = 0.7


# ===============================================================
# GIMBAL MM -> DERECE DONUSUMU
# ===============================================================

DEG_PER_MM_X = 1.0 / 5.0
DEG_PER_MM_Y = 1.0 / 2.37


# ===============================================================
# BORESIGHT R ICIN BASLANGIC
#
# R = gimbal koordinat sistemi -> kamera koordinat sistemi
#
# Baslangicta iki eksenin hizali oldugu varsayiliyor.
# Fit daha sonra gercek montaj hatasini buluyor.
# ===============================================================

R_GIMBAL_TO_CAMERA_INITIAL = np.eye(3, dtype=float)


# ===============================================================
# LAZER / OPTIK BILGILERI
# ===============================================================

LASER_WAVELENGTH_NM = 633.0
PINHOLE_DIAM_UM = 150.0


# ===============================================================
# KAYIT KLASORU
# ===============================================================

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

formatted_time_stamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

SAVE_DIR = os.path.join(
    BASE_DIR,
    f"olcum_{GRID_SIZE_X}x{GRID_SIZE_Y}_tarih_{formatted_time_stamp}"
)

print(f"[*] Kayıt Klasörü: {SAVE_DIR}")


# ===============================================================
# KIRINIM TAHMINI
# ===============================================================

def print_diffraction_estimate():

    lam_um = LASER_WAVELENGTH_NM / 1000.0

    theta_rad = 1.22 * lam_um / PINHOLE_DIAM_UM

    print(
        f"[bilgi] 633nm / 150um pinhole -> "
        f"kirinim yari-acisi ~ "
        f"{theta_rad*1000:.2f} mrad "
        f"(~{np.degrees(theta_rad):.3f} deg)."
    )

    print(
        "[bilgi] Bu bilgi optik sistemin fiziksel davranisini "
        "aciklamak icindir; projeksiyon modeli T kullanmiyor."
    )


# ===============================================================
# GIMBAL / MARLIN YARDIMCI FONKSIYONLARI
# ===============================================================

def send_and_wait(ser, cmd):

    ser.reset_input_buffer()

    ser.write((cmd + '\n').encode())
    ser.flush()

    while True:

        line = ser.readline().decode(
            'utf-8',
            errors='ignore'
        ).strip()

        if line and line.startswith("X:"):
            print(f"  [M114 Gimbal Pos] {line}")

        if line.startswith("ok"):
            break


def wait_move_done(ser):

    send_and_wait(ser, 'M400')
    send_and_wait(ser, 'M114')


def round_stepper(val):

    return round(val / STEP_RES) * STEP_RES


# ===============================================================
# GRID PLAN
# ===============================================================

def build_scan_plan(grid_x, grid_y, limit_x, limit_y):

    span_x = limit_x * 2.0
    span_y = limit_y * 2.0

    step_x = round_stepper(
        span_x / max(1, grid_x - 1)
    )

    step_y = round_stepper(
        span_y / max(1, grid_y - 1)
    )

    print(
        f"[*] Otomatik Step Boyutları: "
        f"X = {step_x:.3f} mm, "
        f"Y = {step_y:.3f} mm"
    )

    half_x = round_stepper(
        (grid_x - 1) * step_x / 2.0
    )

    half_y = round_stepper(
        (grid_y - 1) * step_y / 2.0
    )

    x_positions = [
        round_stepper(
            -half_x + i * step_x
        )
        for i in range(grid_x)
    ]

    y_positions = [
        round_stepper(
            +half_y - j * step_y
        )
        for j in range(grid_y)
    ]

    plan = []

    current_x = x_positions[0]
    current_y = y_positions[0]

    point_no = 1

    for row in range(grid_y):

        y_target = y_positions[row]

        row_x = x_positions

        for step_idx, x_target in enumerate(row_x):

            dx = round_stepper(
                x_target - current_x
            )

            dy = round_stepper(
                y_target - current_y
            )

            actual_col = step_idx

            label = (
                f"P{point_no:03d}_"
                f"{x_target:.3f}_"
                f"{y_target:.3f}"
            )

            plan.append({
                "dx": dx,
                "dy": dy,
                "label": label,
                "mm_x": x_target,
                "mm_y": y_target,
                "row": row,
                "col": actual_col
            })

            current_x = x_target
            current_y = y_target

            point_no += 1

    return (
        plan,
        x_positions[0],
        y_positions[0]
    )


# ===============================================================
# KAMERA
# ===============================================================

class VimbaCamera:

    def __init__(self):

        self._vmb_cm = VmbSystem.get_instance()
        self._vmb_cm.__enter__()

        cams = self._vmb_cm.get_all_cameras()

        if not cams:
            raise RuntimeError(
                "Vimba: hiçbir kamera bulunamadı."
            )

        self.cam = cams[0]
        self.cam.__enter__()

        supported_formats = self.cam.get_pixel_formats()

        if PixelFormat.Mono8 in supported_formats:

            self.cam.set_pixel_format(
                PixelFormat.Mono8
            )

        elif PixelFormat.Mono16 in supported_formats:

            self.cam.set_pixel_format(
                PixelFormat.Mono16
            )

        self._current_exposure_us = 20000.0

        try:

            self.cam.ExposureTimeAbs.set(
                self._current_exposure_us
            )

        except Exception:

            try:

                self.cam.ExposureTime.set(
                    self._current_exposure_us
                )

            except Exception:
                pass

    def _set_exposure(self, exposure_us):

        exposure_us = float(
            np.clip(
                exposure_us,
                20,
                1_000_000
            )
        )

        try:

            self.cam.ExposureTimeAbs.set(
                exposure_us
            )

        except Exception:

            try:

                self.cam.ExposureTime.set(
                    exposure_us
                )

            except Exception:
                pass

        self._current_exposure_us = exposure_us

    def capture_photo_auto_exposure(
        self,
        target_max=200,
        max_tries=6,
        saturation_limit=250,
        min_signal=30
    ):

        img = None

        for attempt in range(max_tries):

            img = self.capture_photo()

            peak = float(img.max())

            if peak >= saturation_limit:

                self._set_exposure(
                    self._current_exposure_us * 0.4
                )

                continue

            elif peak < min_signal:

                self._set_exposure(
                    self._current_exposure_us * 2.5
                )

                continue

            else:

                scale = (
                    target_max /
                    max(peak, 1.0)
                )

                scale = float(
                    np.clip(
                        scale,
                        0.5,
                        2.0
                    )
                )

                self._set_exposure(
                    self._current_exposure_us *
                    scale
                )

                return img

        return img

    def capture_photo(self):

        frame = self.cam.get_frame()

        if (
            frame.get_pixel_format()
            not in OPENCV_PIXEL_FORMATS
        ):

            src_fmt = frame.get_pixel_format()

            convertible_formats = (
                src_fmt.get_convertible_formats()
            )

            if PixelFormat.Mono8 in convertible_formats:

                frame = frame.convert_pixel_format(
                    PixelFormat.Mono8
                )

            elif PixelFormat.Mono16 in convertible_formats:

                frame = frame.convert_pixel_format(
                    PixelFormat.Mono16
                )

            else:

                raise RuntimeError(
                    "Kamera piksel formati "
                    "donusturulemiyor."
                )

        img = frame.as_opencv_image()

        if img.ndim == 3:

            if img.shape[2] == 1:

                img = img[:, :, 0]

            elif img.shape[2] in (3, 4):

                img = cv2.cvtColor(
                    img,
                    cv2.COLOR_BGR2GRAY
                )

        return img

    def close(self):

        try:

            self.cam.__exit__(
                None,
                None,
                None
            )

        finally:

            self._vmb_cm.__exit__(
                None,
                None,
                None
            )


# ===============================================================
# UTF-8 GORUNTU KAYDI
# ===============================================================

def cv2_imwrite_utf8(filename, img):

    ext = os.path.splitext(filename)[1]

    retval, buffer = cv2.imencode(
        ext,
        img
    )

    if retval:

        with open(filename, 'wb') as f:
            f.write(buffer)


# ===============================================================
# LAZER MERKEZI TESPITI
#
# YENI MANTIK:
#
# 1) HICBIR threshold yok
# 2) HICBIR area filtresi yok
# 3) HICBIR morphology yok
# 4) HICBIR edge filtresi yok
# 5) HICBIR FOUND / NOT_FOUND yok
# 6) HICBIR NaN uretmiyoruz
#
# Her karede:
#
#       en parlak piksel
#              |
#              v
#       kucuk ROI
#              |
#              v
#       arka plan tahmini
#              |
#              v
#       agirlikli centroid
#
# ===============================================================

def find_laser_centroid(img):

    img_f = img.astype(
        np.float64
    )

    # -----------------------------------------------------------
    # 1. EN PARLAK NOKTAYI BUL
    # -----------------------------------------------------------

    _, peak_value, _, max_loc = cv2.minMaxLoc(
        img_f
    )

    peak_value = float(peak_value)

    x_peak, y_peak = max_loc

    # -----------------------------------------------------------
    # 2. MAX NOKTA ETRAFINDA KUCUK ROI
    # -----------------------------------------------------------

    radius = 8

    x1 = max(
        0,
        x_peak - radius
    )

    x2 = min(
        img.shape[1],
        x_peak + radius + 1
    )

    y1 = max(
        0,
        y_peak - radius
    )

    y2 = min(
        img.shape[0],
        y_peak + radius + 1
    )

    roi = img_f[
        y1:y2,
        x1:x2
    ]

    # -----------------------------------------------------------
    # 3. BACKGROUND
    #
    # ROI'nin kenar piksellerinden medyan aliyoruz.
    # Bu herhangi bir "lazer bulundu/bulunmadi" karari degildir.
    # Sadece centroid agirliklarini daha stabil hale getirir.
    # -----------------------------------------------------------

    if roi.size == 0:

        return (
            float(x_peak),
            float(y_peak),
            peak_value,
            0.0
        )

    border_pixels = np.concatenate([
        roi[0, :],
        roi[-1, :],
        roi[:, 0],
        roi[:, -1]
    ])

    background = float(
        np.median(border_pixels)
    )

    # -----------------------------------------------------------
    # 4. BACKGROUND CIKAR
    # -----------------------------------------------------------

    weights = roi - background

    weights = np.maximum(
        weights,
        0.0
    )

    # -----------------------------------------------------------
    # 5. SUBPIXEL CENTROID
    # -----------------------------------------------------------

    weight_sum = float(
        weights.sum()
    )

    if weight_sum <= 1e-12:

        # Teorik olarak burada bile NaN yok.
        # En parlak piksel dogrudan kullaniliyor.

        return (
            float(x_peak),
            float(y_peak),
            peak_value,
            background
        )

    yy, xx = np.indices(
        roi.shape
    )

    cx_local = float(
        (xx * weights).sum()
        / weight_sum
    )

    cy_local = float(
        (yy * weights).sum()
        / weight_sum
    )

    cx = cx_local + x1
    cy = cy_local + y1

    return (
        float(cx),
        float(cy),
        peak_value,
        background
    )


# ===============================================================
# TARAMA
# ===============================================================

def run_scan(
    ser,
    camera,
    plan
):

    results = []

    WINDOW_NAME = (
        "Lazer Tarama - Canli Goruntu"
    )

    cv2.namedWindow(
        WINDOW_NAME,
        cv2.WINDOW_NORMAL
    )

    cv2.resizeWindow(
        WINDOW_NAME,
        900,
        700
    )

    for step in plan:

        parts = []

        if step["dx"] != 0:

            parts.append(
                f"X{step['dx']:.3f}"
            )

        if step["dy"] != 0:

            parts.append(
                f"Y{step['dy']:.3f}"
            )

        if parts:

            send_and_wait(
                ser,
                "G1 " +
                " ".join(parts) +
                f" F{FEEDRATE}"
            )

            wait_move_done(ser)

            wait_time = (
                ROW_START_DWELL_SEC
                if step["col"] == 0
                else DWELL_SEC
            )

            print(
                f"[{step['label']}] "
                f"Hedefe ulasildi, "
                f"titresim icin "
                f"{wait_time} sn bekleniyor..."
            )

            time.sleep(
                wait_time
            )

        else:

            print(
                f"[{step['label']}] "
                f"(Baslangic noktasindayiz, "
                f"beklemeden foto cekiliyor...)"
            )

        # -------------------------------------------------------
        # FOTO
        # -------------------------------------------------------

        img = camera.capture_photo_auto_exposure()

        fname = os.path.join(
            SAVE_DIR,
            f"{step['label']}.png"
        )

        cv2_imwrite_utf8(
            fname,
            img
        )

        # -------------------------------------------------------
        # LAZER TESPITI
        # -------------------------------------------------------

        xu, yu, peak, background = (
            find_laser_centroid(img)
        )

        print(
            f"  -> piksel = "
            f"({xu:.3f}, {yu:.3f}) "
            f"| peak={peak:.1f} "
            f"| bg={background:.1f}"
        )

        # -------------------------------------------------------
        # CANLI GORUNTU
        # -------------------------------------------------------

        display = (
            cv2.cvtColor(
                img,
                cv2.COLOR_GRAY2BGR
            )
            if img.ndim == 2
            else img.copy()
        )

        cv2.drawMarker(
            display,
            (
                int(round(xu)),
                int(round(yu))
            ),
            (0, 0, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=25,
            thickness=2
        )

        cv2.putText(
            display,
            (
                f"{step['label']} "
                f"px=({xu:.1f},{yu:.1f}) "
                f"peak={peak:.0f}"
            ),
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        cv2.imshow(
            WINDOW_NAME,
            display
        )

        cv2.waitKey(1)

        # -------------------------------------------------------
        # SONUC
        # -------------------------------------------------------

        results.append({
            **step,
            "xu": xu,
            "yu": yu,
            "peak": peak,
            "background": background,
            "image_path": fname
        })

    cv2.destroyWindow(
        WINDOW_NAME
    )

    return results


# ===============================================================
# ROTASYON MATRİSLERİ
# ===============================================================

def rotation_x_rad(a):

    c = np.cos(a)
    s = np.sin(a)

    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c]
    ], dtype=float)


def rotation_y_rad(a):

    c = np.cos(a)
    s = np.sin(a)

    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c]
    ], dtype=float)


def rotation_z_rad(a):

    c = np.cos(a)
    s = np.sin(a)

    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1]
    ], dtype=float)


# ===============================================================
# GLOBAL BORESIGHT R
#
# R = Rz(rz) Ry(ry) Rx(rx)
#
# Buradaki rx/ry/rz:
#   gimbal eksenleri ile kamera optik ekseni arasindaki
#   SABIT montaj hatasidir.
#
# Tarama boyunca degismez.
# ===============================================================

def rotation_from_rpy_deg(
    rx_deg,
    ry_deg,
    rz_deg
):

    rx, ry, rz = np.radians([
        rx_deg,
        ry_deg,
        rz_deg
    ])

    return (
        rotation_z_rad(rz)
        @ rotation_y_rad(ry)
        @ rotation_x_rad(rx)
    )


# ===============================================================
# GIMBAL KOMUTLARI -> ACI
# ===============================================================

def gimbal_angles_from_result(r):

    alpha_deg = (
        r["mm_x"] *
        DEG_PER_MM_X
    )

    beta_deg = (
        r["mm_y"] *
        DEG_PER_MM_Y
    )

    return (
        alpha_deg,
        beta_deg
    )


# ===============================================================
# COUPLED GIMBAL MODEL
#
# ISTENEN YENI FORMUL:
#
#       x' = tan(alpha)
#
#       y' = -tan(beta) / cos(alpha)
#
# Bu iki normalized koordinat dogrudan bir yon vektoru olarak:
#
#       d = [x', y', 1]
#
# seklinde yazilabilir.
#
# Sonra global boresight R uygulanir.
#
# ===============================================================

def coupled_normalized_coordinates(
    alpha_deg,
    beta_deg
):

    alpha = np.radians(
        float(alpha_deg)
    )

    beta = np.radians(
        float(beta_deg)
    )

    cos_alpha = np.cos(
        alpha
    )

    if abs(cos_alpha) < 1e-10:

        raise RuntimeError(
            "cos(alpha) sifira cok yakin."
        )

    xprime = np.tan(
        alpha
    )

    yprime = (
        -np.tan(beta)
        / cos_alpha
    )

    return (
        xprime,
        yprime
    )


# ===============================================================
# COUPLED MODEL -> GIMBAL YON VEKTORU
# ===============================================================

def gimbal_laser_direction(
    theta_x_deg,
    theta_y_deg
):

    xprime, yprime = (
        coupled_normalized_coordinates(
            theta_x_deg,
            theta_y_deg
        )
    )

    d = np.array([
        xprime,
        yprime,
        1.0
    ], dtype=float)

    n = np.linalg.norm(d)

    if n <= 1e-12:

        raise RuntimeError(
            "Gimbal yon vektoru sifira cok yakin."
        )

    return d / n


# ===============================================================
# GLOBAL BORESIGHT R UYGULA
# ===============================================================

def transform_direction_gimbal_to_camera(
    direction_g,
    R
):

    direction_g = np.asarray(
        direction_g,
        dtype=float
    )

    direction_c = (
        R @ direction_g
    )

    n = np.linalg.norm(
        direction_c
    )

    if n <= 1e-12:

        raise RuntimeError(
            "Kamera yon vektoru sifira cok yakin."
        )

    return (
        direction_c / n
    )


# ===============================================================
# COUPLED MODEL + GLOBAL R
#
# Butun deney icin TEK R vardir.
# ===============================================================

def normalized_coordinates_from_R(
    results,
    R
):

    xp = []
    yp = []

    for r in results:

        alpha_deg, beta_deg = (
            gimbal_angles_from_result(r)
        )

        direction_g = (
            gimbal_laser_direction(
                alpha_deg,
                beta_deg
            )
        )

        direction_c = (
            transform_direction_gimbal_to_camera(
                direction_g,
                R
            )
        )

        if direction_c[2] <= 1e-10:

            raise RuntimeError(
                f"{r['label']} icin lazer "
                f"kameranin arkasina dusuyor. "
                f"Boresight R kontrol edilmeli."
            )

        x_c = (
            direction_c[0]
            / direction_c[2]
        )

        y_c = (
            direction_c[1]
            / direction_c[2]
        )

        xp.append(x_c)
        yp.append(y_c)

        r["dir_x_c"] = float(
            direction_c[0]
        )

        r["dir_y_c"] = float(
            direction_c[1]
        )

        r["dir_z_c"] = float(
            direction_c[2]
        )

    return (
        np.asarray(xp),
        np.asarray(yp)
    )


# ===============================================================
# KAMERA PROJEKSIYONU
#
# x', y'
#      |
#      v
# radial distortion
#      |
#      v
# tangential distortion
#      |
#      v
# fx, fy, cx, cy
#      |
#      v
# u, v
# ===============================================================

def project(
    params,
    xprime,
    yprime
):

    (
        fx,
        fy,
        cx,
        cy,
        k1,
        k2,
        p1,
        p2
    ) = params

    r2 = (
        xprime**2
        +
        yprime**2
    )

    radial = (
        1.0
        + k1 * r2
        + k2 * r2**2
    )

    x_dist = (
        xprime * radial
        + 2 * p1 * xprime * yprime
        + p2 * (
            r2
            + 2 * xprime**2
        )
    )

    y_dist = (
        yprime * radial
        + p1 * (
            r2
            + 2 * yprime**2
        )
        + 2 * p2 * xprime * yprime
    )

    u = (
        fx * x_dist
        + cx
    )

    v = (
        fy * y_dist
        + cy
    )

    return u, v


# ===============================================================
# ESKI MODEL
#
# Sadece referans icin:
#
#       x' = tan(alpha)
#       y' = tan(beta)
#
# ===============================================================

def old_model_normalized_coordinates(
    results
):

    tx = np.radians(
        np.array([
            r["mm_x"] *
            DEG_PER_MM_X
            for r in results
        ], dtype=float)
    )

    ty = np.radians(
        np.array([
            r["mm_y"] *
            DEG_PER_MM_Y
            for r in results
        ], dtype=float)
    )

    return (
        np.tan(tx),
        np.tan(ty)
    )


# ===============================================================
# ESKI MODEL FIT
# ===============================================================

def fit_old_model(results):

    valid = [
        r for r in results
        if np.isfinite(r["xu"])
        and np.isfinite(r["yu"])
    ]

    if len(valid) < 8:

        raise RuntimeError(
            "Eski model icin yeterli nokta yok."
        )

    xp, yp = (
        old_model_normalized_coordinates(
            valid
        )
    )

    u = np.array([
        r["xu"] for r in valid
    ])

    v = np.array([
        r["yu"] for r in valid
    ])

    def residual(p):

        up, vp = project(
            p,
            xp,
            yp
        )

        return np.concatenate([
            u - up,
            v - vp
        ])

    x0 = np.array([
        2290,
        2290,
        1024,
        1024,
        0,
        0,
        0,
        0
    ], dtype=float)

    result = least_squares(
        residual,
        x0,
        method='trf',
        x_scale='jac',
        max_nfev=30000
    )

    p = result.x

    rms = float(
        np.sqrt(
            np.mean(
                result.fun**2
            )
        )
    )

    return {
        "fx": p[0],
        "fy": p[1],
        "cx": p[2],
        "cy": p[3],
        "k1": p[4],
        "k2": p[5],
        "p1": p[6],
        "p2": p[7],
        "rms_px": rms,
        "n_points": len(valid)
    }


# ===============================================================
# YENI MODEL:
#
# COUPLED GIMBAL + TEK GLOBAL BORESIGHT R
#
# PARAMETRELER:
#
#   fx fy cx cy
#   k1 k2 p1 p2
#   rx ry rz
#
# TOPLAM = 11 PARAMETRE
#
# T YOK.
# ===============================================================

def fit_model_R_only(
    results,
    optimize_R=True
):

    valid = [
        r for r in results
        if np.isfinite(r["xu"])
        and np.isfinite(r["yu"])
    ]

    if len(valid) < 12:

        raise RuntimeError(
            "R-only fit icin yeterli nokta yok."
        )

    u = np.array([
        r["xu"] for r in valid
    ], dtype=float)

    v = np.array([
        r["yu"] for r in valid
    ], dtype=float)

    # -----------------------------------------------------------
    # Kamera baslangic tahmini
    # -----------------------------------------------------------

    cam0 = np.array([
        1800.0,     # fx
        1800.0,     # fy
        1024.0,     # cx
        1024.0,     # cy
        0.0,        # k1
        0.0,        # k2
        0.0,        # p1
        0.0         # p2
    ], dtype=float)

    # -----------------------------------------------------------
    # SADECE R FIT
    #
    # Intrinsic parametreleri ilk asamada sabit tutuluyor.
    # -----------------------------------------------------------

    def residual_R(rpy):

        R = rotation_from_rpy_deg(
            rpy[0],
            rpy[1],
            rpy[2]
        )

        xp, yp = (
            normalized_coordinates_from_R(
                valid,
                R
            )
        )

        up = (
            cam0[0] * xp
            + cam0[2]
        )

        vp = (
            cam0[1] * yp
            + cam0[3]
        )

        return np.concatenate([
            u - up,
            v - vp
        ])

    rpy0 = np.zeros(
        3,
        dtype=float
    )

    if optimize_R:

        stage1 = least_squares(
            residual_R,
            rpy0,
            bounds=(
                [-5, -5, -5],
                [5, 5, 5]
            ),
            method='trf',
            x_scale='jac',
            max_nfev=20000
        )

        rpy1 = stage1.x

    else:

        rpy1 = rpy0

    R1 = rotation_from_rpy_deg(
        *rpy1
    )

    # -----------------------------------------------------------
    # R SABITKEN KAMERA PARAMETRELERINI FIT ET
    # -----------------------------------------------------------

    xp, yp = (
        normalized_coordinates_from_R(
            valid,
            R1
        )
    )

    def residual_cam(cam):

        up, vp = project(
            cam,
            xp,
            yp
        )

        return np.concatenate([
            u - up,
            v - vp
        ])

    cam_fit = least_squares(
        residual_cam,
        cam0,
        bounds=(
            [
                1200,
                1200,
                900,
                900,
                -3,
                -3,
                -0.5,
                -0.5
            ],
            [
                3000,
                3000,
                1148,
                1148,
                3,
                3,
                0.5,
                0.5
            ]
        ),
        x_scale='jac',
        method='trf',
        max_nfev=40000
    )

    # -----------------------------------------------------------
    # SON ORTAK FIT
    #
    # fx fy cx cy k1 k2 p1 p2 rx ry rz
    #
    # T YOK.
    # -----------------------------------------------------------

    joint0 = np.r_[
        cam_fit.x,
        rpy1
    ]

    def residual_joint(p):

        cam = p[:8]

        rpy = p[8:11]

        R = rotation_from_rpy_deg(
            rpy[0],
            rpy[1],
            rpy[2]
        )

        xp2, yp2 = (
            normalized_coordinates_from_R(
                valid,
                R
            )
        )

        up, vp = project(
            cam,
            xp2,
            yp2
        )

        return np.concatenate([
            u - up,
            v - vp
        ])

    lower = [
        1200,
        1200,
        900,
        900,
        -3,
        -3,
        -0.5,
        -0.5,
        -5,
        -5,
        -5
    ]

    upper = [
        3000,
        3000,
        1148,
        1148,
        3,
        3,
        0.5,
        0.5,
        5,
        5,
        5
    ]

    final = least_squares(
        residual_joint,
        joint0,
        bounds=(
            lower,
            upper
        ),
        method='trf',
        x_scale='jac',
        max_nfev=80000
    )

    vals = final.x

    (
        fx,
        fy,
        cx,
        cy,
        k1,
        k2,
        p1,
        p2,
        rx,
        ry,
        rz
    ) = vals

    R = rotation_from_rpy_deg(
        rx,
        ry,
        rz
    )

    rms = float(
        np.sqrt(
            np.mean(
                final.fun**2
            )
        )
    )

    return {
        "fx": fx,
        "fy": fy,
        "cx": cx,
        "cy": cy,
        "k1": k1,
        "k2": k2,
        "p1": p1,
        "p2": p2,

        "rx_deg": rx,
        "ry_deg": ry,
        "rz_deg": rz,

        "R": R,

        "rms_px": rms,
        "n_points": len(valid)
    }


# ===============================================================
# SATIR HIZALAMA KONTROLU
# ===============================================================

def row_alignment_check(
    results,
    model_name,
    predict_fn
):

    valid = [
        r for r in results
        if np.isfinite(r["xu"])
        and np.isfinite(r["yu"])
    ]

    rows = sorted(
        set(
            r["row"]
            for r in valid
        )
    )

    first_row = rows[0]
    last_row = rows[-1]

    def first_col_point(row_no):

        cand = [
            r for r in valid
            if r["row"] == row_no
        ]

        return min(
            cand,
            key=lambda r: r["col"]
        )

    r1 = first_col_point(
        first_row
    )

    r2 = first_col_point(
        last_row
    )

    dv_meas = (
        r2["yu"]
        - r1["yu"]
    )

    up1, vp1 = predict_fn(
        r1["mm_x"],
        r1["mm_y"]
    )

    up2, vp2 = predict_fn(
        r2["mm_x"],
        r2["mm_y"]
    )

    dv_pred = (
        vp2 - vp1
    )

    print(
        f"\n--- Satir hizalama kontrolu "
        f"[{model_name}] ---"
    )

    print(
        f"OLCULEN dv = "
        f"{dv_meas:+.3f} px"
    )

    print(
        f"MODEL dv = "
        f"{dv_pred:+.3f} px"
    )

    return (
        dv_meas,
        dv_pred
    )


# ===============================================================
# COMPOSITE MESH
# ===============================================================

def save_composite(
    results,
    img_shape,
    out_path
):

    composite = np.zeros(
        img_shape,
        dtype=np.uint8
    )

    composite_color = cv2.cvtColor(
        composite,
        cv2.COLOR_GRAY2BGR
    )

    point_map = {
        (r["row"], r["col"]): r
        for r in results
    }

    for r in results:

        if not np.isfinite(
            r["xu"]
        ):
            continue

        pt = (
            int(round(r["xu"])),
            int(round(r["yu"]))
        )

        for d_row, d_col in [
            (0, 1),
            (1, 0)
        ]:

            neighbor = point_map.get(
                (
                    r["row"] + d_row,
                    r["col"] + d_col
                )
            )

            if (
                neighbor is not None
                and np.isfinite(
                    neighbor["xu"]
                )
            ):

                pt2 = (
                    int(round(
                        neighbor["xu"]
                    )),
                    int(round(
                        neighbor["yu"]
                    ))
                )

                cv2.line(
                    composite_color,
                    pt,
                    pt2,
                    (0, 255, 255),
                    1
                )

        cv2.drawMarker(
            composite_color,
            pt,
            (0, 0, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=14,
            thickness=2
        )

    cv2_imwrite_utf8(
        out_path,
        composite_color
    )


# ===============================================================
# DISTORTION COMPARISON
#
# YESIL = AYNI FIT MODELINDEN gelen IDEAL NOKTALAR
# SARI  = GERCEK OLCUM
#
# Burada eski/kuplajsiz formul YOK.
# Ideal noktalar da fit edilen coupled + R modelinden geliyor.
# ===============================================================

def save_distortion_comparison(
    results,
    fit_params,
    img_shape,
    out_path
):

    canvas = np.zeros(
        img_shape,
        dtype=np.uint8
    )

    color = cv2.cvtColor(
        canvas,
        cv2.COLOR_GRAY2BGR
    )

    point_map = {
        (r["row"], r["col"]): r
        for r in results
    }

    cam = [
        fit_params[k]
        for k in (
            "fx",
            "fy",
            "cx",
            "cy",
            "k1",
            "k2",
            "p1",
            "p2"
        )
    ]

    R = fit_params["R"]

    # -----------------------------------------------------------
    # IDEAL / DISTORTIONSUZ NOKTALAR
    #
    # AYNI COUPLED + R MODEL
    # -----------------------------------------------------------

    for r in results:

        if not np.isfinite(
            r["xu"]
        ):
            continue

        alpha_deg, beta_deg = (
            gimbal_angles_from_result(r)
        )

        dg = gimbal_laser_direction(
            alpha_deg,
            beta_deg
        )

        dc = (
            transform_direction_gimbal_to_camera(
                dg,
                R
            )
        )

        xp = (
            dc[0] /
            dc[2]
        )

        yp = (
            dc[1] /
            dc[2]
        )

        # distortion uygulanmadan
        # onceki ideal projection

        ideal = (
            int(round(
                cam[0] * xp
                + cam[2]
            )),
            int(round(
                cam[1] * yp
                + cam[3]
            ))
        )

        r["ideal_pt"] = ideal

    # -----------------------------------------------------------
    # IDEAL GRID
    # -----------------------------------------------------------

    for r in results:

        if "ideal_pt" not in r:
            continue

        pt = r["ideal_pt"]

        for dr, dc in [
            (0, 1),
            (1, 0)
        ]:

            nb = point_map.get(
                (
                    r["row"] + dr,
                    r["col"] + dc
                )
            )

            if (
                nb is not None
                and "ideal_pt" in nb
            ):

                cv2.line(
                    color,
                    pt,
                    nb["ideal_pt"],
                    (0, 255, 0),
                    1
                )

    # -----------------------------------------------------------
    # GERCEK OLCUM
    # -----------------------------------------------------------

    for r in results:

        if not np.isfinite(
            r["xu"]
        ):
            continue

        pt = (
            int(round(r["xu"])),
            int(round(r["yu"]))
        )

        for dr, dc in [
            (0, 1),
            (1, 0)
        ]:

            nb = point_map.get(
                (
                    r["row"] + dr,
                    r["col"] + dc
                )
            )

            if (
                nb is not None
                and np.isfinite(
                    nb["xu"]
                )
            ):

                pt2 = (
                    int(round(
                        nb["xu"]
                    )),
                    int(round(
                        nb["yu"]
                    ))
                )

                cv2.line(
                    color,
                    pt,
                    pt2,
                    (0, 255, 255),
                    1
                )

        cv2.drawMarker(
            color,
            pt,
            (0, 0, 255),
            cv2.MARKER_CROSS,
            10,
            1
        )

    cv2.putText(
        color,
        "Yesil=Coupled+R ideal | Sari=olcum",
        (20, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        1
    )

    cv2_imwrite_utf8(
        out_path,
        color
    )


# ===============================================================
# PHI RESIDUAL
#
# phi = atan2(y-cy, x-cx)
#
# Ama burada residual:
#
#       measured - predicted
#
# olarak hesaplanir.
#
# Eger residual phi'ye bagli sistematik bir dalga yapiyorsa,
# modelde eksik yonsel bir etki olabilir.
# ===============================================================

def plot_phi_residual(
    results,
    fit_params,
    out_path
):

    import matplotlib.pyplot as plt

    valid = [
        r for r in results
        if np.isfinite(r["xu"])
        and np.isfinite(r["yu"])
    ]

    if len(valid) < 10:
        print(
            "[uyari] Phi residual icin yeterli nokta yok."
        )
        return

    cam = [
        fit_params[k]
        for k in (
            "fx",
            "fy",
            "cx",
            "cy",
            "k1",
            "k2",
            "p1",
            "p2"
        )
    ]

    R = fit_params["R"]

    phi = []
    residual = []

    for r in valid:

        alpha_deg, beta_deg = (
            gimbal_angles_from_result(r)
        )

        dg = gimbal_laser_direction(
            alpha_deg,
            beta_deg
        )

        dc = transform_direction_gimbal_to_camera(
            dg,
            R
        )

        xp = dc[0] / dc[2]
        yp = dc[1] / dc[2]

        up, vp = project(
            cam,
            xp,
            yp
        )

        dx = r["xu"] - up
        dy = r["yu"] - vp

        angle = np.arctan2(
            r["yu"] - fit_params["cy"],
            r["xu"] - fit_params["cx"]
        )

        # radial residual
        radial_residual = (
            dx * np.cos(angle)
            +
            dy * np.sin(angle)
        )

        phi.append(
            np.degrees(angle)
        )

        residual.append(
            radial_residual
        )

    phi = np.asarray(phi)
    residual = np.asarray(residual)

    order = np.argsort(phi)

    phi = phi[order]
    residual = residual[order]

    plt.figure(
        figsize=(10, 6)
    )

    plt.scatter(
        phi,
        residual,
        s=8
    )

    plt.axhline(
        0,
        linewidth=1
    )

    plt.xlabel(
        "phi [deg]"
    )

    plt.ylabel(
        "Radial residual [px]"
    )

    plt.title(
        "Phi-Residual / Yon Bağımlılığı Kontrolü"
    )

    plt.grid(
        True,
        alpha=0.3
    )

    plt.tight_layout()

    plt.savefig(
        out_path,
        dpi=200
    )

    plt.close()

    print(
        f"[*] Phi residual kaydedildi: "
        f"{out_path}"
    )


# ===============================================================
# DISTORTION PERCENTAGE
#
# Distorsiyon:
#
#       distorted radius / ideal radius - 1
#
# yuzde olarak:
#
#       distortion(%) =
#           100 * (rd/r - 1)
#
# k1, k2 kullanilarak:
#
#       rd/r = 1 + k1*r^2 + k2*r^4
#
# Datasheet referansi:
#
#       -3.1 %
#
# ===============================================================

def compute_distortion_percentage(
    fit_params,
    r_values
):

    k1 = fit_params["k1"]
    k2 = fit_params["k2"]

    r_values = np.asarray(
        r_values,
        dtype=float
    )

    distortion = (
        k1 * r_values**2
        +
        k2 * r_values**4
    )

    return (
        100.0 *
        distortion
    )


def plot_distortion_curve(
    fit_params,
    out_path
):

    import matplotlib.pyplot as plt

    # -----------------------------------------------------------
    # Datasetten kullanilabilecek normalized radius araligi
    # -----------------------------------------------------------

    r_max = 0.0

    for r in np.linspace(
        0.0,
        1.0,
        1000
    ):

        pass

    # Daha fiziksel olarak:
    # kamera goruntusunun kose radiusunu normalize et.

    fx = fit_params["fx"]
    fy = fit_params["fy"]
    cx = fit_params["cx"]
    cy = fit_params["cy"]

    corners = [
        (0, 0),
        (2048, 0),
        (0, 2048),
        (2048, 2048)
    ]

    radii = []

    for u, v in corners:

        x = (
            u - cx
        ) / fx

        y = (
            v - cy
        ) / fy

        radii.append(
            np.sqrt(
                x*x + y*y
            )
        )

    r_max = max(
        radii
    )

    r_values = np.linspace(
        0.0,
        r_max,
        500
    )

    distortion = (
        compute_distortion_percentage(
            fit_params,
            r_values
        )
    )

    datasheet_value = -3.1

    # -----------------------------------------------------------
    # EN UZAK NOKTADAKI FIT
    # -----------------------------------------------------------

    fit_edge = float(
        distortion[-1]
    )

    difference = (
        fit_edge
        - datasheet_value
    )

    print(
        "\n=== DISTORSION KARSILASTIRMASI ==="
    )

    print(
        f"Fit edge distortion = "
        f"{fit_edge:+.4f} %"
    )

    print(
        f"Datasheet reference = "
        f"{datasheet_value:+.4f} %"
    )

    print(
        f"Mutlak fark = "
        f"{abs(difference):.4f} percentage point"
    )

    # -----------------------------------------------------------
    # GRAFIK
    # -----------------------------------------------------------

    plt.figure(
        figsize=(10, 6)
    )

    plt.plot(
        r_values,
        distortion,
        label="Fitted k1/k2"
    )

    plt.axhline(
        datasheet_value,
        linestyle="--",
        label="Datasheet: -3.1%"
    )

    plt.xlabel(
        "Normalized radius r"
    )

    plt.ylabel(
        "Distortion [%]"
    )

    plt.title(
        "Radyal Distorsiyon Egrisi"
    )

    plt.grid(
        True,
        alpha=0.3
    )

    plt.legend()

    plt.tight_layout()

    plt.savefig(
        out_path,
        dpi=200
    )

    plt.close()

    return {
        "fit_edge_percent": fit_edge,
        "datasheet_percent": datasheet_value,
        "difference_percent_point": difference
    }


# ===============================================================
# MODEL PREDICT
# ===============================================================

def predict_with_fit(
    mm_x,
    mm_y,
    fit_params
):

    alpha_deg = (
        mm_x *
        DEG_PER_MM_X
    )

    beta_deg = (
        mm_y *
        DEG_PER_MM_Y
    )

    dg = gimbal_laser_direction(
        alpha_deg,
        beta_deg
    )

    dc = (
        transform_direction_gimbal_to_camera(
            dg,
            fit_params["R"]
        )
    )

    cam = [
        fit_params[k]
        for k in (
            "fx",
            "fy",
            "cx",
            "cy",
            "k1",
            "k2",
            "p1",
            "p2"
        )
    ]

    return project(
        cam,
        dc[0] / dc[2],
        dc[1] / dc[2]
    )


# ===============================================================
# MODEL RAPORU
# ===============================================================

def save_model_report(
    results,
    old_fit,
    new_fit,
    dv_info,
    distortion_info,
    out_path
):

    with open(
        out_path,
        "w",
        encoding="utf-8"
    ) as f:

        f.write(
            "GIMBAL -> KAMERA MODEL RAPORU\n"
        )

        f.write(
            "==============================\n\n"
        )

        f.write(
            f"Nokta sayisi: "
            f"{new_fit['n_points']}\n\n"
        )

        f.write(
            "MODEL KARSILASTIRMASI\n"
        )

        f.write(
            "---------------------\n"
        )

        f.write(
            f"Eski bagimsiz tan(theta) RMS = "
            f"{old_fit['rms_px']:.6f} px\n"
        )

        f.write(
            f"Yeni coupled + global R RMS = "
            f"{new_fit['rms_px']:.6f} px\n\n"
        )

        f.write(
            "YENI MODEL MATEMATIGI\n"
        )

        f.write(
            "---------------------\n"
        )

        f.write(
            "x' = tan(alpha)\n"
        )

        f.write(
            "y' = -tan(beta) / cos(alpha)\n\n"
        )

        f.write(
            "p_cam = R * d_gimbal\n"
        )

        f.write(
            "T KULLANILMIYOR.\n\n"
        )

        f.write(
            "BORESIGHT R\n"
        )

        f.write(
            "-----------\n"
        )

        f.write(
            np.array2string(
                new_fit["R"],
                precision=10,
                suppress_small=True
            )
        )

        f.write("\n\n")

        f.write(
            "Boresight Euler acilari [deg]:\n"
        )

        f.write(
            f"rx = "
            f"{new_fit['rx_deg']:.9f}\n"
        )

        f.write(
            f"ry = "
            f"{new_fit['ry_deg']:.9f}\n"
        )

        f.write(
            f"rz = "
            f"{new_fit['rz_deg']:.9f}\n\n"
        )

        f.write(
            "KAMERA PARAMETRELERI\n"
        )

        f.write(
            "--------------------\n"
        )

        for k in (
            "fx",
            "fy",
            "cx",
            "cy",
            "k1",
            "k2",
            "p1",
            "p2"
        ):

            f.write(
                f"{k} = "
                f"{new_fit[k]:.12f}\n"
            )

        f.write("\n")

        f.write(
            "SATIR HIZALAMA\n"
        )

        f.write(
            "--------------\n"
        )

        f.write(
            f"Olculen dv = "
            f"{dv_info['meas']:+.6f} px\n"
        )

        f.write(
            f"Yeni model dv = "
            f"{dv_info['new_pred']:+.6f} px\n\n"
        )

        f.write(
            "DISTORSIYON\n"
        )

        f.write(
            "-----------\n"
        )

        f.write(
            f"Fit edge = "
            f"{distortion_info['fit_edge_percent']:+.6f} %\n"
        )

        f.write(
            f"Datasheet = "
            f"{distortion_info['datasheet_percent']:+.6f} %\n"
        )

        f.write(
            f"Fark = "
            f"{distortion_info['difference_percent_point']:+.6f} "
            f"percentage point\n\n"
        )

        f.write(
            "OPTIK BILGI\n"
        )

        f.write(
            "-----------\n"
        )

        lam_um = (
            LASER_WAVELENGTH_NM /
            1000.0
        )

        theta_rad = (
            1.22 *
            lam_um /
            PINHOLE_DIAM_UM
        )

        f.write(
            f"lambda = "
            f"{LASER_WAVELENGTH_NM:.1f} nm\n"
        )

        f.write(
            f"Pinhole = "
            f"{PINHOLE_DIAM_UM:.1f} um\n"
        )

        f.write(
            f"Diffraction half-angle = "
            f"{theta_rad*1000:.4f} mrad\n"
        )

        f.write(
            f"Diffraction half-angle = "
            f"{np.degrees(theta_rad):.6f} deg\n"
        )

        f.write("\n")

        f.write(
            "NOT:\n"
        )

        f.write(
            "Bu modelde Zhang kalibrasyonundaki gibi "
            "her kare icin ayri R/T yoktur.\n"
        )

        f.write(
            "Tek bir global boresight R kullanilir.\n"
        )

        f.write(
            "T parametresi projeksiyon modeline dahil edilmemistir.\n"
        )


# ===============================================================
# CSV
# ===============================================================

def save_csv(
    results,
    out_path
):

    fieldnames = [
        "label",
        "mm_x",
        "mm_y",
        "row",
        "col",
        "xu",
        "yu",
        "peak",
        "background",
        "image_path",
        "dir_x_c",
        "dir_y_c",
        "dir_z_c"
    ]

    with open(
        out_path,
        "w",
        newline="",
        encoding="utf-8"
    ) as f:

        writer = csv.DictWriter(
            f,
            fieldnames=fieldnames,
            extrasaction="ignore"
        )

        writer.writeheader()

        for r in results:

            writer.writerow({
                k: r.get(k, "")
                for k in fieldnames
            })


# ===============================================================
# KALIBRASYON CSV
# ===============================================================

def save_calibration_csv(
    fit,
    old_fit,
    distortion_info,
    out_path
):

    with open(
        out_path,
        "w",
        newline="",
        encoding="utf-8"
    ) as f:

        writer = csv.writer(f)

        writer.writerow([
            "parameter",
            "value"
        ])

        for k in (
            "fx",
            "fy",
            "cx",
            "cy",
            "k1",
            "k2",
            "p1",
            "p2",
            "rx_deg",
            "ry_deg",
            "rz_deg",
            "rms_px",
            "n_points"
        ):

            writer.writerow([
                k,
                fit[k]
            ])

        writer.writerow([
            "R_00",
            fit["R"][0, 0]
        ])

        writer.writerow([
            "R_01",
            fit["R"][0, 1]
        ])

        writer.writerow([
            "R_02",
            fit["R"][0, 2]
        ])

        writer.writerow([
            "R_10",
            fit["R"][1, 0]
        ])

        writer.writerow([
            "R_11",
            fit["R"][1, 1]
        ])

        writer.writerow([
            "R_12",
            fit["R"][1, 2]
        ])

        writer.writerow([
            "R_20",
            fit["R"][2, 0]
        ])

        writer.writerow([
            "R_21",
            fit["R"][2, 1]
        ])

        writer.writerow([
            "R_22",
            fit["R"][2, 2]
        ])

        writer.writerow([
            "old_model_rms_px",
            old_fit["rms_px"]
        ])

        writer.writerow([
            "distortion_fit_edge_percent",
            distortion_info[
                "fit_edge_percent"
            ]
        ])

        writer.writerow([
            "distortion_datasheet_percent",
            distortion_info[
                "datasheet_percent"
            ]
        ])

        writer.writerow([
            "distortion_difference_percentage_point",
            distortion_info[
                "difference_percent_point"
            ]
        ])


# ===============================================================
# ANA PROGRAM
# ===============================================================

def main():

    os.makedirs(
        SAVE_DIR,
        exist_ok=True
    )

    print_diffraction_estimate()

    # -----------------------------------------------------------
    # SERVO
    # -----------------------------------------------------------

    ser = serial.Serial(
        PORT,
        BAUD,
        timeout=1
    )

    # -----------------------------------------------------------
    # KAMERA
    # -----------------------------------------------------------

    camera = VimbaCamera()

    ser.reset_input_buffer()

    # -----------------------------------------------------------
    # HOME
    # -----------------------------------------------------------

    print(
        f"Homing bekleniyor "
        f"({HOMING_WAIT_SEC} sn)..."
    )

    time.sleep(
        HOMING_WAIT_SEC
    )

    send_and_wait(
        ser,
        'G92 X0 Y0'
    )

    send_and_wait(
        ser,
        'G91'
    )

    # -----------------------------------------------------------
    # GRID
    # -----------------------------------------------------------

    plan, start_x, start_y = (
        build_scan_plan(
            GRID_SIZE_X,
            GRID_SIZE_Y,
            SAFE_LIMIT_X,
            SAFE_LIMIT_Y
        )
    )

    print(
        "\n=== OFFSET: ilk baslangic "
        f"noktasina gidiliyor "
        f"(X={start_x:.3f}, "
        f"Y={start_y:.3f}) ==="
    )

    send_and_wait(
        ser,
        f"G1 X{start_x:.3f} "
        f"Y{start_y:.3f} "
        f"F{FEEDRATE}"
    )

    wait_move_done(
        ser
    )

    print(
        f"Offset'e ulaşıldı. "
        f"Titreşimin sönmesi için "
        f"{INITIAL_DWELL_SEC} sn bekleniyor..."
    )

    time.sleep(
        INITIAL_DWELL_SEC
    )

    # -----------------------------------------------------------
    # GLOBAL R BASLANGIC
    # -----------------------------------------------------------

    print(
        "\n=== GLOBAL BORESIGHT R "
        "BASLANGIC TAHMINI ==="
    )

    print(
        "R0 ="
    )

    print(
        R_GIMBAL_TO_CAMERA_INITIAL
    )

    print(
        "\nT YOK."
    )

    print(
        "Modelde yalnızca tek global R "
        "fit edilecek."
    )

    # -----------------------------------------------------------
    # TARAMA
    # -----------------------------------------------------------

    print(
        f"\n=== "
        f"{GRID_SIZE_X}x{GRID_SIZE_Y} "
        f"tarama basliyor "
        f"({len(plan)} nokta) ===\n"
    )

    results = run_scan(
        ser,
        camera,
        plan
    )

    print(
        f"\n=== Tarama tamamlandı, "
        f"{len(results)} nokta işlendi ===\n"
    )

    # -----------------------------------------------------------
    # HAM CSV
    # -----------------------------------------------------------

    save_csv(
        results,
        os.path.join(
            SAVE_DIR,
            "sonuclar.csv"
        )
    )

    # -----------------------------------------------------------
    # COMPOSITE
    # -----------------------------------------------------------

    sample_img = (
        camera.capture_photo()
    )

    img_shape = (
        sample_img.shape
    )

    save_composite(
        results,
        img_shape,
        os.path.join(
            SAVE_DIR,
            "composite_mesh.png"
        )
    )

    try:

        # -------------------------------------------------------
        # ESKI MODEL
        # -------------------------------------------------------

        old_fit = fit_old_model(
            results
        )

        # -------------------------------------------------------
        # YENI COUPLED + GLOBAL R MODEL
        # -------------------------------------------------------

        new_fit = fit_model_R_only(
            results,
            optimize_R=True
        )

        print(
            "\n=== MODEL KARSILASTIRMASI ==="
        )

        print(
            f"1) Eski tan(theta) RMS "
            f"= {old_fit['rms_px']:.4f} px"
        )

        print(
            f"2) Coupled + global R RMS "
            f"= {new_fit['rms_px']:.4f} px"
        )

        # -------------------------------------------------------
        # PREDICT FUNCTION
        # -------------------------------------------------------

        def new_predict(
            mm_x,
            mm_y
        ):

            return predict_with_fit(
                mm_x,
                mm_y,
                new_fit
            )

        # -------------------------------------------------------
        # SATIR HIZALAMA
        # -------------------------------------------------------

        dv_meas, dv_new = (
            row_alignment_check(
                results,
                "Coupled + global R",
                new_predict
            )
        )

        # -------------------------------------------------------
        # SONUCLARI YAZDIR
        # -------------------------------------------------------

        print(
            "\n=== YENI KALIBRASYON SONUCU ==="
        )

        print(
            f"fx = "
            f"{new_fit['fx']:.6f}"
        )

        print(
            f"fy = "
            f"{new_fit['fy']:.6f}"
        )

        print(
            f"cx = "
            f"{new_fit['cx']:.6f}"
        )

        print(
            f"cy = "
            f"{new_fit['cy']:.6f}"
        )

        print(
            f"k1 = "
            f"{new_fit['k1']:.9f}"
        )

        print(
            f"k2 = "
            f"{new_fit['k2']:.9f}"
        )

        print(
            f"p1 = "
            f"{new_fit['p1']:.9f}"
        )

        print(
            f"p2 = "
            f"{new_fit['p2']:.9f}"
        )

        print(
            f"rx = "
            f"{new_fit['rx_deg']:.9f} deg"
        )

        print(
            f"ry = "
            f"{new_fit['ry_deg']:.9f} deg"
        )

        print(
            f"rz = "
            f"{new_fit['rz_deg']:.9f} deg"
        )

        print(
            "\nR ="
        )

        print(
            new_fit["R"]
        )

        print(
            f"\nRMS = "
            f"{new_fit['rms_px']:.6f} px"
        )

        # -------------------------------------------------------
        # DISTORTION COMPARISON
        # -------------------------------------------------------

        save_distortion_comparison(
            results,
            new_fit,
            img_shape,
            os.path.join(
                SAVE_DIR,
                "R_model_vs_olcum.png"
            )
        )

        # -------------------------------------------------------
        # PHI RESIDUAL
        # -------------------------------------------------------

        plot_phi_residual(
            results,
            new_fit,
            os.path.join(
                SAVE_DIR,
                "phi_residual.png"
            )
        )

        # -------------------------------------------------------
        # DISTORTION %
        # -------------------------------------------------------

        distortion_info = (
            plot_distortion_curve(
                new_fit,
                os.path.join(
                    SAVE_DIR,
                    "distortion_percent.png"
                )
            )
        )

        # -------------------------------------------------------
        # RAPOR
        # -------------------------------------------------------

        save_model_report(
            results,
            old_fit,
            new_fit,
            {
                "meas": dv_meas,
                "new_pred": dv_new
            },
            distortion_info,
            os.path.join(
                SAVE_DIR,
                "kalibrasyon_raporu.txt"
            )
        )

        # -------------------------------------------------------
        # KALIBRASYON CSV
        # -------------------------------------------------------

        save_calibration_csv(
            new_fit,
            old_fit,
            distortion_info,
            os.path.join(
                SAVE_DIR,
                "kalibrasyon_sonuc.csv"
            )
        )

        print(
            "\n=== DOSYALAR OLUSTURULDU ==="
        )

        print(
            "sonuclar.csv"
        )

        print(
            "kalibrasyon_sonuc.csv"
        )

        print(
            "kalibrasyon_raporu.txt"
        )

        print(
            "composite_mesh.png"
        )

        print(
            "R_model_vs_olcum.png"
        )

        print(
            "phi_residual.png"
        )

        print(
            "distortion_percent.png"
        )

    except RuntimeError as e:

        print(
            f"\nFit yapılamadı: {e}"
        )

    # -----------------------------------------------------------
    # GIMBAL DURDUR
    # -----------------------------------------------------------

    send_and_wait(
        ser,
        'M17'
    )

    send_and_wait(
        ser,
        'M84 S0'
    )

    print(
        "\nGimbal son konumda bekliyor. "
        "Kapatmak için CTRL+C."
    )

    try:

        while True:

            time.sleep(1)

    except KeyboardInterrupt:

        print(
            "Kapatılıyor..."
        )

    finally:

        camera.close()
        ser.close()


# ===============================================================
# START
# ===============================================================

if __name__ == "__main__":

    main()
