# Insight 360 LiDAR spatial mapper

An embedded 3D‐mapping system that uses the MSP-EXP432E401Y microcontroller and VL53L1X time‑of‑flight sensor mounted on a stepper motor to perform 360° scans in a vertical plane, capture fixed‑interval slices along the orthogonal axis, and stream the resulting point‑cloud data to a PC for real‑time reconstruction and visualization.

<br><br>

## 3D Scan

[3D image1](../ToF360%20LiDAR%20Spatial%20Mapper/Images/scan_image1.png)
<br>
[3D Image2](/Images/scan_image2.png)
<br>
<br>

## Location

[Location](/Images/hallwayImage.png)
<br><br>

## Hardware setup

[HardwareSetup](/Images/hardware_image.png)

<br><br>

## Specifications

- MCU bus clock: 14 MHz
- Sensor & scan: VL53L1X ToF on stepper motor scanning every 11.25 degrees on vertical‑plane (y‑z) distance mapping
- Displacement slices: manual X‑axis steps (30 cm) to build full 3D point cloud
- Start/stop control: three momentary push buttons (one to start motor/data collection, one to stop motor/output data, and one to reset motor/rescan)

<br>

Status indicators: (assigned)

- Measurement status LED → PN0
- UART‑TX activity LED → PN1
- Additional/troubleshoot LED → PF4

<br>

Comm protocols:

- I²C between MCU ↔ ToF sensor
- UART from MCU → PC for serial point‑cloud output
