
Tübitak Uzay GZGSG
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

---
*Developed for high-precision Electro-Optical Engineering and R&D applications*
