# Insight 360 LiDAR spatial mapper

An embedded 3D‐mapping system that uses the MSP-EXP432E401Y microcontroller and VL53L1X time‑of‑flight sensor mounted on a stepper motor to perform 360° scans in a vertical plane, capture fixed‑interval slices along the orthogonal axis, and stream the resulting point‑cloud data to a PC for real‑time reconstruction and visualization.
<br>

## 3D Scan

![scan_image2](https://github.com/user-attachments/assets/a06112d5-188a-404d-a04a-7a28f107874c)

![scan_image1](https://github.com/user-attachments/assets/0effd7ee-4177-4258-998a-eabaf5aa280c)
<br>

## Location

![hallwayImage](https://github.com/user-attachments/assets/6fc73dfe-0496-4e4b-ad57-df842c389229)
<br>

## Hardware setup

![hardware_image](https://github.com/user-attachments/assets/be79a866-e69d-4742-b3ac-7111aed3f17b)
<br>

## Specifications

- MCU bus clock: 14 MHz
- Sensor & scan: VL53L1X ToF on stepper motor scanning every 11.25 degrees on vertical‑plane (y‑z) distance mapping
- Displacement slices: manual X‑axis steps (30 cm) to build full 3D point cloud
- Start/stop control: three momentary push buttons (one to start motor/data collection, one to stop motor/output data, and one to reset motor/rescan)

<br>

Status indicators (assigned):

- Measurement status LED → PN0
- UART‑TX activity LED → PN1
- Additional/troubleshoot LED → PF4

Comm protocols:

- I²C between MCU ↔ ToF sensor
- UART from MCU → PC for serial point‑cloud output
