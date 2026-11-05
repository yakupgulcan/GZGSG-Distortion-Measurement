import time
import cv2
import numpy as np
from vmbpy import OPENCV_PIXEL_FORMATS, PixelFormat, VmbSystem

# Mono8 icin baslangic ayarlari
MIN_AREA = 12
MAX_AREA = 5000
MIN_CONTRAST = 25.0
MIN_PEAK = 50.0
NOISE_SIGMA_MULT = 6.0
BORDER_MARGIN = 2
SEARCH_BLUR = 9
CENTROID_BLUR = 5
MAX_JUMP_PX = 300.0


def _background_and_noise(img_f):
    background = float(np.median(img_f))
    mad = float(np.median(np.abs(img_f - background)))
    sigma = max(1.0, 1.4826 * mad)
    return background, sigma


def find_laser_centroid(img, previous_centroid=None, max_jump_px=None):
    """
    Lazer bulunamazsa RuntimeError verir.
    Tek bir parlak pikseli otomatik olarak lazer kabul etmez.
    """
    if img is None or img.size == 0:
        raise RuntimeError("LASER NOT FOUND | gecersiz goruntu")

    img_f = img.astype(np.float32)
    background, noise_sigma = _background_and_noise(img_f)

    required_contrast = max(
        MIN_CONTRAST,
        NOISE_SIGMA_MULT * noise_sigma
    )
    threshold = max(MIN_PEAK, background + required_contrast)

    smooth = cv2.GaussianBlur(img_f, (SEARCH_BLUR, SEARCH_BLUR), 0)
    binary = (smooth >= threshold).astype(np.uint8)

    kernel = np.ones((3, 3), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

    n, labels, stats, centroids = cv2.connectedComponentsWithStats(
        binary, connectivity=8
    )

    candidates = []

    for idx in range(1, n):
        area = int(stats[idx, cv2.CC_STAT_AREA])
        if area < MIN_AREA or area > MAX_AREA:
            continue

        x = int(stats[idx, cv2.CC_STAT_LEFT])
        y = int(stats[idx, cv2.CC_STAT_TOP])
        w = int(stats[idx, cv2.CC_STAT_WIDTH])
        h = int(stats[idx, cv2.CC_STAT_HEIGHT])

        # Kenara dayanan adaylari reddet.
        if (
            x <= BORDER_MARGIN or
            y <= BORDER_MARGIN or
            x + w >= img.shape[1] - BORDER_MARGIN or
            y + h >= img.shape[0] - BORDER_MARGIN
        ):
            continue

        mask_component = labels == idx
        peak = float(img_f[mask_component].max())
        mean_intensity = float(img_f[mask_component].mean())
        contrast = peak - background

        if peak < MIN_PEAK or contrast < MIN_CONTRAST:
            continue

        aspect = max(w / max(h, 1), h / max(w, 1))
        if aspect > 6.0:
            continue

        cx0, cy0 = centroids[idx]

        distance_penalty = 0.0
        if previous_centroid is not None and max_jump_px is not None:
            distance = float(np.hypot(
                cx0 - previous_centroid[0],
                cy0 - previous_centroid[1]
            ))
            if distance > max_jump_px:
                continue
            distance_penalty = distance / max_jump_px

        score = (
            peak
            + 0.20 * mean_intensity
            + 0.5 * np.log1p(area)
            - 20.0 * distance_penalty
        )

        candidates.append({
            "area": area,
            "peak": peak,
            "mean": mean_intensity,
            "contrast": contrast,
            "cx0": float(cx0),
            "cy0": float(cy0),
            "score": float(score),
            "bbox": (x, y, w, h),
            "threshold": threshold,
            "background": background,
            "noise_sigma": noise_sigma,
        })

    if not candidates:
        raise RuntimeError(
            f"LASER NOT FOUND | uygun blob yok | "
            f"peak={img_f.max():.1f}, threshold={threshold:.1f}"
        )

    best = max(candidates, key=lambda c: c["score"])

    x, y, w, h = best["bbox"]
    roi = cv2.GaussianBlur(
        img_f[y:y+h, x:x+w],
        (CENTROID_BLUR, CENTROID_BLUR),
        0
    )

    local_threshold = max(
        best["threshold"],
        best["background"] + 0.35 * best["contrast"]
    )

    mask = roi >= local_threshold
    if not np.any(mask):
        raise RuntimeError("LASER NOT FOUND | centroid maskesi bos")

    ys, xs = np.nonzero(mask)
    weights = np.maximum(
        roi[ys, xs] - best["background"],
        0.0
    )

    weight_sum = float(weights.sum())
    if weight_sum <= 0:
        raise RuntimeError("LASER NOT FOUND | centroid agirligi sifir")

    cx = float((xs * weights).sum() / weight_sum + x)
    cy = float((ys * weights).sum() / weight_sum + y)

    info = {
        **best,
        "centroid": (cx, cy),
        "n_candidates": len(candidates),
    }
    return cx, cy, info


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


def build_display_image(img, centroid=None, info=None, message=None):
    if img.dtype != np.uint8:
        base = cv2.normalize(
            img, None, 0, 255, cv2.NORM_MINMAX
        ).astype(np.uint8)
    else:
        base = img

    display = cv2.cvtColor(base, cv2.COLOR_GRAY2BGR)

    if centroid is not None:
        cx, cy = centroid
        p = (int(round(cx)), int(round(cy)))

        cv2.drawMarker(
            display, p, (0, 0, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=25, thickness=2
        )
        cv2.circle(display, p, 12, (0, 255, 255), 1)

        cv2.putText(
            display,
            f"LASER FOUND | x={cx:.2f}, y={cy:.2f}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8, (0, 255, 0), 2
        )

        if info is not None:
            cv2.putText(
                display,
                f"Area={info['area']} Peak={info['peak']:.1f} "
                f"Thr={info['threshold']:.1f}",
                (20, 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65, (0, 255, 0), 2
            )
    else:
        cv2.putText(
            display,
            message or "LASER NOT FOUND",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8, (0, 0, 255), 2
        )

    return display


def main():
    window_name = "Laser Centroid"
    camera = VimbaCamera()

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 900, 700)

    try:
        while True:
            img = camera.capture_photo()

            try:
                cx, cy, info = find_laser_centroid(img)
                display = build_display_image(
                    img, (cx, cy), info=info
                )

                print(
                    f"LASER FOUND | x={cx:.2f}, y={cy:.2f} | "
                    f"area={info['area']} | peak={info['peak']:.1f} | "
                    f"threshold={info['threshold']:.1f}"
                )

            except RuntimeError as exc:
                print(str(exc))
                display = build_display_image(
                    img, message=str(exc)
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
