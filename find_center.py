import time
import cv2
import numpy as np
from vmbpy import OPENCV_PIXEL_FORMATS, PixelFormat, VmbSystem


# ---------------------------------------------------------------
# LAZER MERKEZİ TESPİTİ
# ---------------------------------------------------------------
def _get_speckle_blobs(img_f, rel_thr=0.5, min_area=3):
    """
    Görüntüdeki her ayrık parlak beneği (speckle) tek tek tespit eder ve
    her biri için (merkez_x, merkez_y, alan) döndürür.
    """
    blurred = cv2.GaussianBlur(img_f, (5, 5), 0)
    thr = rel_thr * blurred.max()
    mask = (blurred >= thr).astype(np.uint8)
    n, _, stats, centroids = cv2.connectedComponentsWithStats(mask)

    blobs = []
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        if area >= min_area:
            cx, cy = centroids[i]
            blobs.append((float(cx), float(cy), float(area)))
    return blobs


def _count_speckles(img_f, rel_thr=0.5, min_area=3):
    """
    Görüntüdeki ayrık parlak nokta (speckle) sayısını sayar.
    Diffraction paterni oluştuğunda tek bir blob yerine onlarca ayrı
    parlak beneğe bölünür; bu fonksiyon bunu tespit etmek için kullanılır.
    """
    return len(_get_speckle_blobs(img_f, rel_thr=rel_thr, min_area=min_area))


def _robust_envelope_center(blobs, trim_factor=2.5, min_keep_ratio=0.5, max_iter=4):
    """
    Diffraction paterninin ana (yoğun) beneğinden oluşan bulutunun merkezini
    bulur. Sağ üst köşedeki sensör yansıması, alt sol köşedeki fringe deseni
    gibi ANA BULUTTAN UZAK, TEKİL parlak lekelerin merkezi bozmasını
    önlemek için medyan tabanlı, aykırı-değer eleyen (robust) bir yöntem
    kullanır:

      1) Başlangıç tahmini olarak tüm benek merkezlerinin MEDYANI alınır
         (medyan, ortalamanın aksine birkaç uzak aykırı değerden neredeyse
         hiç etkilenmez).
      2) Bu tahmine olan uzaklıklara bakılır, medyan uzaklığın
         `trim_factor` katından uzak olan benekler (yani ana bulutun
         parçası olmayan tekil lekeler) elenir.
      3) Kalan beneklerin alan-ağırlıklı ortalaması ile merkez güncellenir.
      4) Birkaç kez tekrarlanarak merkez stabilize edilir.

    Döndürür: (cx, cy, ana_bulutun_yaricapi)
    """
    pts = np.array([(b[0], b[1]) for b in blobs], dtype=np.float64)
    weights = np.array([b[2] for b in blobs], dtype=np.float64)

    c = np.median(pts, axis=0)
    keep = np.ones(len(pts), dtype=bool)

    for _ in range(max_iter):
        dists = np.linalg.norm(pts - c, axis=1)
        med_dist = np.median(dists)
        if med_dist <= 0:
            break

        new_keep = dists <= (med_dist * trim_factor)
        # Cok fazla benegi elemeyelim (guvenlik)
        if new_keep.sum() < max(4, int(len(pts) * min_keep_ratio)):
            break

        keep = new_keep
        pts_k = pts[keep]
        w_k = weights[keep]
        c_new = (pts_k * w_k[:, None]).sum(axis=0) / w_k.sum()

        if np.linalg.norm(c_new - c) < 0.5:
            c = c_new
            break
        c = c_new

    dists = np.linalg.norm(pts - c, axis=1)
    kept_dists = dists[keep] if keep.any() else dists
    envelope_radius = float(np.percentile(kept_dists, 90)) if len(kept_dists) else 0.0

    return float(c[0]), float(c[1]), envelope_radius


def _find_simple_laser_centroid(img):
    """
    ESKİ ALGORİTMA (diffraction YOKKEN kullanılır - tek nokta lazer).
    Güçlü Gaussian Blur ile saçılmalara karşı dirençli ROI + connected
    components + intensity weighted centroid algoritması.
    """
    img_f = img.astype(np.float32)

    blurred_for_max = cv2.GaussianBlur(img_f, (41, 41), 0)
    _, max_val, _, max_loc = cv2.minMaxLoc(blurred_for_max)
    if max_val <= 0:
        raise RuntimeError("Lazer noktasi bulunamadi.")

    roi_size = 60
    x0, y0 = max_loc
    x1 = max(0, x0 - roi_size)
    x2 = min(img.shape[1], x0 + roi_size + 1)
    y1 = max(0, y0 - roi_size)
    y2 = min(img.shape[0], y0 + roi_size + 1)

    roi = img_f[y1:y2, x1:x2]
    roi = cv2.GaussianBlur(roi, (5, 5), 0)

    thr = 0.5 * roi.max()
    binary = (roi >= thr).astype("uint8")

    n, labels, stats, _ = cv2.connectedComponentsWithStats(binary)
    if n <= 1:
        raise RuntimeError("Lazer blob'u bulunamadi.")

    idx = 1 + stats[1:, cv2.CC_STAT_AREA].argmax()
    mask = labels == idx

    ys, xs = mask.nonzero()
    w = roi[ys, xs]
    weight_sum = w.sum()

    if weight_sum <= 0:
        raise RuntimeError("Lazer centroid hesabi yapilamadi.")

    cx = (xs * w).sum() / weight_sum + x1
    cy = (ys * w).sum() / weight_sum + y1

    return float(cx), float(cy)


def _find_diffraction_center(img):
    """
    YENİ ALGORİTMA (diffraction paterni VARKEN kullanılır).

    Diffraction oluştuğunda gerçek merkez, en parlak beneğin olduğu yer
    DEĞİL; paternin tam ortasındaki, çevresi parlak fakat kendisi KARANLIK
    olan sönümlenme noktasıdır (fotoğrafta gördüğünüz o küçük siyah nokta).

    Adımlar:
      1) `_robust_envelope_center` ile, uzaktaki tekil parlak lekelerden
         etkilenmeyen, ana speckle bulutunun GÜVENİLİR kaba merkezini bul.
      2) Bu merkezin etrafında (bulutun tipik yarıçapına göre ölçeklenmiş)
         küçük bir pencere içinde, "parlak çevre + karanlık iç" yapısına
         uyan bölgeyi ara ve o bölgenin yoğunluk-ağırlıklı merkezini
         gerçek merkez olarak döndür.
    """
    img_f = img.astype(np.float32)

    blobs = _get_speckle_blobs(img_f)
    if len(blobs) < 4:
        raise RuntimeError("Diffraction merkezi icin yeterli benek yok.")

    cx_coarse, cy_coarse, env_radius = _robust_envelope_center(blobs)
    if env_radius <= 0:
        env_radius = 60.0

    # 2) Kaba merkezin etrafinda, bulutun tipik yaricapina gore olceklenmis
    #    kucuk bir pencerede karanlik noktayi ara.
    search_r = max(15, int(env_radius * 0.4))
    x0 = max(0, int(cx_coarse - search_r))
    x1 = min(img.shape[1], int(cx_coarse + search_r + 1))
    y0 = max(0, int(cy_coarse - search_r))
    y1 = min(img.shape[0], int(cy_coarse + search_r + 1))

    sub = img_f[y0:y1, x0:x1]

    # "Karanlik delik" tespiti: sadece en karanlik pikseli aramak, kenar/
    # arka plan gibi zaten karanlik olan bolgeleri de yanlislikla secebilir.
    # Bunun yerine, her pikselin GENIS cevresindeki ortalama parlakligi
    # (buyuk blur) kendi degeriyle (kucuk blur) kiyaslayip, "cevresi
    # belirgin sekilde parlak ama kendisi karanlik" olan gercek deligi
    # ariyoruz. Kenar/arka plan bolgelerinde hem gecis hem cevre karanlik
    # oldugundan bu fark orada yuksek cikmaz; delik yalnizca patern icinde,
    # parlak halkanin tam ortasinda yuksek fark uretir.
    small_blur = cv2.GaussianBlur(sub, (5, 5), 0)
    large_blur = cv2.GaussianBlur(sub, (31, 31), 0)

    if large_blur.max() <= 0:
        return cx_coarse, cy_coarse

    # Sadece cevresi zaten yeterince parlak olan bolgelerde ariyoruz;
    # aksi halde sonucta karanlik olan bolgeler (arka plan) de aday olur.
    valid = large_blur >= (0.35 * large_blur.max())
    if not valid.any():
        return cx_coarse, cy_coarse

    diff = large_blur - small_blur
    diff_masked = np.where(valid, diff, -np.inf)

    peak_val = diff_masked.max()
    if not np.isfinite(peak_val) or peak_val <= 0:
        return cx_coarse, cy_coarse

    # Tepe degerin bir kismina yakin butun pikselleri toplayip agirlikli
    # merkezini al (tek piksele degil, delik bolgesinin butunune bakariz).
    thr_dark = 0.7 * peak_val
    dark_mask = diff_masked >= thr_dark

    ys, xs = np.nonzero(dark_mask)
    if len(xs) == 0:
        return cx_coarse, cy_coarse

    w = diff_masked[ys, xs]
    cx = (xs * w).sum() / w.sum() + x0
    cy = (ys * w).sum() / w.sum() + y0

    return float(cx), float(cy)


def find_laser_centroid(img, speckle_threshold=6):
    """
    Ana merkez tespit fonksiyonu.

    Önce görüntüde diffraction (kırınım) paterni oluşup oluşmadığına bakar
    (ayrı ayrı parlak benek sayısına göre):
      - Diffraction VARSA -> _find_diffraction_center()
        (paternin ortasındaki siyah noktayı bulan, uzak aykırı parlak
        lekelere dayanıklı yöntem) kullanılır.
      - Diffraction YOKSA (temiz tek nokta lazer) -> eski
        _find_simple_laser_centroid() kullanılır.

    speckle_threshold: Diffraction kabul edilmesi için gereken minimum
    ayrık parlak beneği sayısı. Sahada gerçek verilerle test edip
    ihtiyaca göre 4-10 arası ayarlayabilirsiniz.
    """
    img_f = img.astype(np.float32)
    num_speckles = _count_speckles(img_f)

    if num_speckles >= speckle_threshold:
        try:
            return _find_diffraction_center(img)
        except RuntimeError:
            # Diffraction merkez bulma başarısız olursa eski yönteme geri dön
            return _find_simple_laser_centroid(img)
    else:
        return _find_simple_laser_centroid(img)


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