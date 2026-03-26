# FMCW-Radar-Object-Detection-CFAR-KF
FMCW Radar-based object detection using CFAR and CFAR-Kalman Filter hybrid model for improved target tracking and noise reduction.
## Overview
This project focuses on object detection in FMCW radar data using various CFAR (Constant False Alarm Rate) techniques and enhanced tracking using Kalman Filter-based methods.

The aim is to improve detection accuracy in noisy environments and provide reliable target tracking.

---

## Features
- Implementation of multiple CFAR techniques:
  - CA-CFAR
  - AC-CFAR
  - CAGO-CFAR
  - RSCFAR
- Hybrid models:
  - CFAR + Kalman Filter
  - CFAR + Unscented Kalman Filter (UKF)
  - CFAR + Particle Filter
- Noise reduction and improved detection performance
- Modular MATLAB implementation

---

## Methodology
1. Radar signal preprocessing
2. Application of CFAR algorithms for target detection
3. Tracking detected targets using:
   - Kalman Filter
   - Unscented Kalman Filter
   - Particle Filter
4. Performance comparison between methods

---

## Tech Stack
- MATLAB
- Signal Processing
- Radar Systems
- Estimation & Filtering Techniques

---

## Project Structure
```bash
src/
├── ac_cfar.m
├── cacfar.m
├── cago_cfar.m
├── cfar.m
├── cfarkalman.m
├── cfar_particle_filter.m
├── cfar_unscented_kalman.m
└── rscfar.m
```

---

## Results
The hybrid CFAR-Kalman approach improves detection stability and reduces false alarms compared to traditional CFAR methods.

---


## Highlights
- Robust object detection using CFAR variants
- Enhanced tracking using Kalman and UKF
- Reduced false alarm rate in noisy radar environments
## Future Work
- Integration with real-time radar data
- Deep learning-based target classification
- Optimization for embedded systems

---

## Author
Karshana B G
