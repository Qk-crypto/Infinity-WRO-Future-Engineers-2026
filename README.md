# Infinity — WRO Future Engineers 2026

This repository contains the current documentation package for **Team Infinity** in the **WRO Future Engineers** category.

> This is an **initial version** of the repository. It already includes the current robot photos, 3D design screenshots, component references, and the obstacle challenge source code. More materials (team photos, full wiring diagram, videos, Open Challenge code, and additional CAD files) will be added later.

---

## Team Information

- **Team name:** Infinity
- **Members:** Islam Yekiya, Aslan Nyssanaly, Nurtas Nazarbayev
- **School:** NIS Shymkent Abay
- **Country:** Kazakhstan
- **Category:** WRO Future Engineers

---

## Table of Contents

- [The Robot](#the-robot)
- [Current Hardware](#current-hardware)
- [Robot Photos](#robot-photos)
- [3D Design](#3d-design)
- [Source Code](#source-code)
- [Repository Structure](#repository-structure)
- [Planned Updates](#planned-updates)

---

## The Robot

Our robot is a custom-built autonomous vehicle developed for the WRO Future Engineers category. The current version uses a 3D-printed multi-level chassis, Ackermann-like steering, a dedicated drive motor, camera-based perception, and an IMU-assisted control setup.

At the current stage of documentation, this repository focuses on the following:

- the overall mechanical layout;
- the steering and drive arrangement;
- the electronic components used in the robot;
- the current obstacle challenge code structure;
- real robot photos and 3D model screenshots.

---

## Current Hardware

### Main electronic components

- **Arduino Uno R3** — low-level control
- **OpenMV Cam H7 Plus** — computer vision
- **GY-BNO08X IMU** — orientation sensing
- **BTS7960 motor driver** — DC motor control
- **MG996R servo** — steering actuator
- **GA25-370 gear motor** — drivetrain motor
- **3S Li-Po 3300 mAh battery** — main power source
- **UPS HAT (B)** — auxiliary power module used in the build
- **XL4015 step-down module** — voltage regulation

### Component reference images

| Component | Image |
|---|---|
| Arduino Uno R3 | ![](other/components/arduino-uno-r3.png) |
| OpenMV Cam H7 Plus | ![](other/components/openmv-cam-h7-plus.png) |
| GY-BNO08X IMU | ![](other/components/gy-bno08x-imu.png) |
| BTS7960 Motor Driver | ![](other/components/bts7960-motor-driver.png) |
| MG996R Servo | ![](other/components/mg996r-servo.png) |
| GA25-370 Gear Motor | ![](other/components/ga25-370-gear-motor.png) |
| 3S Li-Po 3300 mAh | ![](other/components/3s-lipo-3300mah.png) |
| UPS HAT (B) | ![](other/components/ups-hat-b.png) |
| XL4015 Step-Down Module | ![](other/components/xl4015-step-down.png) |

---

## Robot Photos

### Competition field image

![](other/competition-field/robot-on-obstacle-field.png)

### Real robot images

| View | Image |
|---|---|
| Front-left view | ![](robot-photos/robot-front-left.jpg) |
| Top view | ![](robot-photos/robot-top.jpg) |
| Left side | ![](robot-photos/robot-left-side.jpg) |
| Right side | ![](robot-photos/robot-right-side.jpg) |
| Rear view | ![](robot-photos/robot-rear.jpg) |
| Front view | ![](robot-photos/robot-front.jpg) |
| Bottom view | ![](robot-photos/robot-bottom.jpg) |

---

## 3D Design

The following screenshots show the current CAD model and some of the most important custom mechanical parts.

| 3D view | Image |
|---|---|
| Steering system (straight) | ![](3d-models/steering-front-straight.png) |
| Steering system (turned) | ![](3d-models/steering-front-turned.png) |
| Drive and differential bottom view | ![](3d-models/drive-and-differential-bottom.png) |
| Overall top layout | ![](3d-models/chassis-top-layout.png) |
| Base plate | ![](3d-models/base-plate-top.png) |
| Middle plate | ![](3d-models/middle-plate-bottom.png) |
| Top plate | ![](3d-models/top-plate-top.png) |
| Chassis plate stack | ![](3d-models/stacked-chassis-plates.png) |
| Servo holder | ![](3d-models/servo-holder.png) |
| Motor coupler | ![](3d-models/motor-coupler.png) |
| Steering linkage assembly | ![](3d-models/steering-linkage-assembly.png) |

---

## Source Code

### Obstacle Challenge

Current code included in this repository:

- `src/obstacle-challenge/openmv/main.py`
- `src/obstacle-challenge/arduino/infinity_obstacle_uno.ino`

### Open Challenge

A placeholder folder for Open Challenge code has already been added:

- `src/open-challenge/`

---

## Repository Structure

```text
Infinity_WRO_FE_2026/
├── 3d-models/
├── commit-log/
├── electrical-diagram/
├── other/
│   ├── competition-field/
│   └── components/
├── robot-photos/
├── src/
│   ├── obstacle-challenge/
│   │   ├── arduino/
│   │   └── openmv/
│   └── open-challenge/
├── team-photos/
├── video/
├── LICENSE
└── README.md
```

---

## Planned Updates

The next update of this repository will include:

- team member photos;
- team group photo;
- full wiring / electrical diagram;
- more detailed hardware explanation;
- Open Challenge code;
- performance videos;
- additional CAD files / exports;
- build steps and assembly instructions.

---

## Notes

This repository is being prepared gradually as part of the team documentation process. The current version is suitable as a strong starting structure and can be expanded with more content later.
