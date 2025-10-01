# ESP32 Wi-Fi CSI Feature Extractor + CSV Logger (PlatformIO)

Turn an ESP32 into a lightweight **Wi-Fi Channel State Information (CSI)** probe for environment sensing.  
The firmware connects (or monitors) Wi-Fi, enables CSI, extracts **per-frame features** from subcarrier I/Q, and logs them to **CSV for 3 minutes** on SPIFFS. A safe ISR→queue design keeps the system stable (no heap or printing inside the CSI callback).

---

## Features

- **CSI capture** while connected to your AP (or in monitor mode)
- **Engineered features per frame**:
  - `cv_sc` – coefficient of variation of |Hₖ| (frequency-selective fading)
  - `ewvar_mean` – temporal exponentially-weighted variance (motion/fading over time)
  - `sfm` – spectral flatness of amplitude (scattering/material selectivity)
  - `slope_amp_dB` – amplitude slope (dB) vs subcarrier (frequency response / materials)
  - `mean_delta_dB` – attenuation vs baseline scene
  - `tau_ns` – group delay from phase slope (multipath delay)
  - `mean_phase_delta` – residual phase after detrending (micro-motion / gestures)
  - `d_est_m` – RSSI-based distance proxy (after calibration)
- **3-minute CSV logging** to SPIFFS + live serial mirroring
- **Filter to your AP** by BSSID for clean single-link data
- **PlatformIO** project ready to build/flash

---

## How it works

- ESP32 CSI is enabled via ESP-IDF APIs (Arduino core).
- CSI callback parses interleaved **int8 I/Q** → amplitude/phase arrays (fixed buffers).
- We compute features (see formulas below) and **enqueue** a compact row to a FreeRTOS queue.
- The main loop dequeues rows, **writes CSV** to SPIFFS, mirrors to Serial, and stops after **3 minutes**.

**CSV columns (per frame):**  
`ts_ms, rssi, chan, cv_sc, ewvar_mean, sfm, slope_amp_dB, mean_delta_dB, tau_ns, mean_phase_delta, d_est_m, mac`

---

## Math (what each feature measures)

- **Frequency selectivity (scattering / materials)**
  - \( \displaystyle \mathrm{CV_{sc}}=\frac{\sqrt{\mathrm{Var}_k(|H_k|)}}{\mathbb{E}_k[|H_k|]} \)
  - \( \displaystyle \text{SFM}=\frac{\exp\big(\frac1N\sum_k \ln|H_k|\big)}{\frac1N\sum_k |H_k|} \)
  - Fit \( A_k^{\mathrm{dB}}=20\log_{10}|H_k| \approx m k + b \Rightarrow \) **slope** \(m\) = `slope_amp_dB`
- **Temporal fading (motion/activity)**  
  EMA: \( \mathrm{EMA}_t=(1-\alpha)\mathrm{EMA}_{t-1}+\alpha x_t \)  
  EWVar: \( \mathrm{EWVar}_t=(1-\alpha)\big(\mathrm{EWVar}_{t-1}+\alpha(x_t-\mathrm{EMA}_{t-1})^2\big) \)  
  We report mean EWVar across subcarriers → `ewvar_mean`.
- **Attenuation vs baseline**  
  \( \Delta A_k=20\log_{10}\frac{|H_k|}{|H_{k,0}|} \), average over \(k\) → `mean_delta_dB`.
- **Group delay (multipath)**  
  \( \phi(f)\approx \phi_0-2\pi f\tau \Rightarrow \tau=-\frac{1}{2\pi}\frac{d\phi}{df} \).  
  Unwrap phase across subcarriers, fit line, convert using \(\Delta f=312.5\) kHz (20 MHz OFDM).
- **Residual phase (micro-motion)**  
  Remove trend \( \tilde\phi_k=\phi_k-(\hat\phi_0+\hat s k) \), average \(|\tilde\phi_k|\) → `mean_phase_delta`.
- **Distance proxy (RSSI)**  
  \( P_r(d)\,[\mathrm{dBm}]=A-10n\log_{10}d \Rightarrow d=10^{\frac{A-P_r}{10n}} \). Calibrate \(A\) (RSSI@1 m) and \(n\).

**Why it’s useful:** motion/presence detection, gesture & micro-motion sensing, obstacle/occlusion tracking, material/path characterization, and coarse ranging/localization.

---

## Quick start

1. **Clone** and open in VS Code + PlatformIO.  
2. In `src/main.cpp`, set `WIFI_SSID` / `WIFI_PASSWORD`.  
3. (Optional) tune: `BASELINE_FRAMES`, `ALPHA`, `RX_REF_dBm`, `PATH_N`, `LOG_DURATION_MS`.  
4. **Build & Upload**; open Serial @ **115200**.  
5. After **3 minutes**, a CSV like `/csi_<boot_ms>.csv` is saved to SPIFFS.

> To download the CSV: use an ESP32 FS plugin/tool or add a small serial dumper.

---

## Repo structure


