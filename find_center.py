import time

import cv2
import numpy as np
from vmbpy import OPENCV_PIXEL_FORMATS, PixelFormat, VmbSystem


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

class VimbaCamera:
    def __init__(self):
        self._vmb_cm = VmbSystem.get_instance()
        self._vmb_cm.__enter__()

        cams = self._vmb_cm.get_all_cameras()
        if not cams:
            raise RuntimeError("Vimba: hicbir kamera bulunamadi.")

        self.cam = cams[0]
        self.cam.__enter__()

        supported_formats = self.cam.get_pixel_formats()
        if PixelFormat.Mono8 in supported_formats:
            self.cam.set_pixel_format(PixelFormat.Mono8)
        elif PixelFormat.Mono16 in supported_formats:
            self.cam.set_pixel_format(PixelFormat.Mono16)

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
                raise RuntimeError(
                    f"Kamera piksel formati {src_fmt}; OpenCV uyumlu formata donusturulemiyor."
                )

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


def build_display_image(img, centroid):
    if img.dtype != np.uint8:
        display_base = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    else:
        display_base = img

    display = cv2.cvtColor(display_base, cv2.COLOR_GRAY2BGR)

    cx, cy = centroid
    px = int(round(cx))
    py = int(round(cy))

    cv2.drawMarker(
        display,
        (px, py),
        (0, 0, 255),
        markerType=cv2.MARKER_CROSS,
        markerSize=25,
        thickness=2,
    )
    cv2.circle(display, (px, py), 12, (0, 255, 255), 1)
    cv2.putText(
        display,
        f"Centroid: ({cx:.2f}, {cy:.2f})",
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2,
    )
    return display


def main():
    window_name = "Laser Centroid"
    camera = VimbaCamera()

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 900, 700)

    last_print = None

    try:
        while True:
            img = camera.capture_photo()

            try:
                centroid = find_laser_centroid(img)
                display = build_display_image(img, centroid)

                rounded = (round(centroid[0], 2), round(centroid[1], 2))
                if rounded != last_print:
                    print(f"Centroid pixel: x={rounded[0]:.2f}, y={rounded[1]:.2f}")
                    last_print = rounded
            except RuntimeError as exc:
                if img.dtype != np.uint8:
                    display_base = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                else:
                    display_base = img

                display = cv2.cvtColor(display_base, cv2.COLOR_GRAY2BGR)
                cv2.putText(
                    display,
                    str(exc),
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2,
                )

            cv2.imshow(window_name, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break

            time.sleep(0.01)
    finally:
        cv2.destroyAllWindows()
        camera.close()


if __name__ == "__main__":
    main()
