# UAV Autonomous Visual Servoing & Target Tracking System

An onboard Computer Vision module designed for Unmanned Aerial Vehicles (UAVs) to perform autonomous landing pad acquisition and payload drop-zone alignment. Built with Python and OpenCV.

## 📌 Features
- **HSV Color Segmentation:** Robust color thresholding minimizing outdoor illumination noise.
- **Morphological Filtering:** Noise suppression using open morphological operators (erosion & dilation).
- **Target Prioritization:** Filters and tracks the largest detected target contour to prevent false positives.
- **Visual Servoing Error Calculation:** Calculates real-time displacement vectors ($\Delta X, \Delta Y$) relative to the UAV's camera optical center.
- **Deadzone / Target-Lock Logic:** Triggers a "LOCKED" state when the target is centered within the tolerance threshold.
- **Lightweight & High-FPS:** Optimized pipeline achieving 30+ FPS, suitable for single-board computers (Raspberry Pi 4 / Nvidia Jetson Nano).

## 🚀 How It Works
1. Optical center $(w/2, h/2)$ represents the UAV's downward gimbal position.
2. The detected target's centroid $(X_t, Y_t)$ is extracted from the largest contour.
3. Spatial error vector is computed:
   $$\Delta X = X_t - X_{center}$$
   $$\Delta Y = Y_{center} - Y_t$$
4. Error values can be piped into a PID controller to generate UAV roll/pitch velocity setpoints via MAVLink.

## 🛠️ Tech Stack & Requirements
- Python 3.8+
- OpenCV (`cv2`)
- NumPy

## 💻 How to Run
```bash
pip install opencv-python numpy
python target_tracking.py
