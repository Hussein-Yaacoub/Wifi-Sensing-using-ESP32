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
  - Coefficient of variation across subcarriers  
    $$
    \mathrm{CV_{sc}}=\frac{\sqrt{\operatorname{Var}_k\!\left(|H_k|\right)}}{\mathbb{E}_k\!\left[|H_k|\right]}
    $$
  - Spectral flatness (geometric / arithmetic mean)  
    $$
    \mathrm{SFM}=\frac{\exp\!\left(\frac{1}{N}\sum_k \ln|H_k|\right)}{\frac{1}{N}\sum_k |H_k|}
    $$
  - Amplitude slope in dB vs subcarrier index  
    $$A_k^{\mathrm{dB}}=20\log_{10}|H_k|\approx m\,k+b \quad\Rightarrow\quad \texttt{slope\_amp\_dB}=m$$

- **Temporal fading (motion/activity)**
  - Exponential moving average  
    $$\mathrm{EMA}_t=(1-\alpha)\,\mathrm{EMA}_{t-1}+\alpha x_t$$
  - Exponentially weighted variance  
    $$\mathrm{EWVar}_t=(1-\alpha)\Big(\mathrm{EWVar}_{t-1}+\alpha\,(x_t-\mathrm{EMA}_{t-1})^2\Big)$$
  - We report mean EWVar across subcarriers → `ewvar_mean`.

- **Attenuation vs baseline**
  $$
  \Delta A_k=20\log_{10}\frac{|H_k|}{|H_{k,0}|}
  \qquad
  \text{and we report } \frac{1}{N}\sum_k \Delta A_k \;=\; \texttt{mean\_delta\_dB}.
  $$

- **Group delay (multipath)**
  $$
  \phi(f)\approx \phi_0-2\pi f\,\tau
  \quad\Rightarrow\quad
  \tau=-\frac{1}{2\pi}\frac{d\phi}{df}
  $$
  (unwrap phase across subcarriers, fit a line vs frequency, use $\Delta f=312.5\,\text{kHz}$ for 20 MHz OFDM).

- **Residual phase (micro-motion)**
  $$
  \tilde\phi_k=\phi_k-(\hat\phi_0+\hat s\,k),
  \qquad
  \text{feature}=\frac{1}{N}\sum_k |\tilde\phi_k|
  \;=\;\texttt{mean\_phase\_delta}.
  $$

- **Distance proxy (RSSI)**
  $$
  P_r(d)\,[\mathrm{dBm}]=A-10n\log_{10} d
  \quad\Rightarrow\quad
  d=10^{\tfrac{A-P_r}{10n}}
  $$
  Calibrate $A$ (RSSI@1 m) and $n$ (path-loss exponent).

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


