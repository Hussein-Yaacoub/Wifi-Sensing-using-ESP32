#include <Arduino.h>
#include <WiFi.h>

extern "C" {
  #include "esp_wifi.h"
  #include "esp_wifi_types.h"
}

//////////////////// USER CONFIG ////////////////////
static const char* WIFI_SSID     = "HUSSEIN'S IPHONE";
static const char* WIFI_PASSWORD = "Ip@d2isnice";

static const bool FILTER_TO_CONNECTED_AP = true;

// Smoothing (for fading variation etc.)
static const float ALPHA = 0.15f;        // EMA factor (0..1); lower = smoother
// Baseline capture
static const int   BASELINE_FRAMES = 50; // frames to average at startup for baseline
// Distance model (calibrate these!)
static const float RX_REF_dBm = -40.0f;  // A: RSSI at 1 m (dBm)
static const float PATH_N     = 2.0f;    // n: path loss exponent (1.6–3.5 indoors)
/////////////////////////////////////////////////////

// Globals
static uint8_t g_connected_bssid[6] = {0};
static bool    g_have_bssid = false;
static int     g_primary_channel = 1;

static bool    g_have_baseline = false;
static int     g_baseline_count = 0;
static float  *g_amp_baseline = nullptr; // |H_k| baseline (allocated when first CSI seen)

// EMAs for fading variation
static float  *g_amp_ema = nullptr;      // per-subcarrier EMA of amplitude
static float  *g_amp_var = nullptr;      // per-subcarrier EW var of amplitude

// Helpers
static void print_mac(const uint8_t mac[6]) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.print(buf);
}
static inline float sqf(float x) { return x*x; }

// Unwrap phase along subcarriers
static void unwrap_phase(float *phi, int N) {
  // unwrap in-place to keep continuity along k
  float offset = 0.0f;
  for (int i = 1; i < N; ++i) {
    float dp = phi[i] - phi[i-1];
    if (dp >  M_PI) offset -= 2.0f * M_PI;
    if (dp < -M_PI) offset += 2.0f * M_PI;
    phi[i] += offset;
  }
}

// Simple linear regression (x=0..N-1 equally spaced). Returns slope and intercept.
static void linreg_y_vs_index(const float *y, int N, float &slope, float &intercept) {
  // slope = cov(k,y)/var(k). For k=0..N-1: mean_k = (N-1)/2, var_k = (N^2-1)/12
  const float mean_k = (N - 1) * 0.5f;
  const float var_k  = (N*N - 1) / 12.0f;
  float cov = 0.0f;
  float mean_y = 0.0f;
  for (int k = 0; k < N; ++k) mean_y += y[k];
  mean_y /= N;
  for (int k = 0; k < N; ++k) cov += (k - mean_k) * (y[k] - mean_y);
  cov /= N;
  slope = cov / var_k;
  intercept = mean_y - slope * mean_k;
}

// Allocate per-N buffers lazily
static void ensure_buffers(int N) {
  if (!g_amp_baseline) {
    g_amp_baseline = (float*)heap_caps_malloc(N*sizeof(float), MALLOC_CAP_8BIT);
    g_amp_ema      = (float*)heap_caps_malloc(N*sizeof(float), MALLOC_CAP_8BIT);
    g_amp_var      = (float*)heap_caps_malloc(N*sizeof(float), MALLOC_CAP_8BIT);
    for (int i=0;i<N;++i){ g_amp_baseline[i]=0; g_amp_ema[i]=0; g_amp_var[i]=0; }
  }
}

// CSI callback — compute features + print CSV per frame
static void IRAM_ATTR csi_rx_cb(void *ctx, wifi_csi_info_t *info) {
  if (!info || !info->buf || info->len == 0) return;
  if (FILTER_TO_CONNECTED_AP && g_have_bssid) {
    if (memcmp(info->mac, g_connected_bssid, 6) != 0) return;
  }

  const int   rssi    = info->rx_ctrl.rssi;           // dBm
  const int   channel = info->rx_ctrl.channel;        // primary chan
  const bool  ht      = info->rx_ctrl.sig_mode == 1;  // 0: non-HT, 1: HT
  (void)ht;

  // Parse interleaved I/Q int8 samples
  const int8_t *buf = (const int8_t*)info->buf;
  const int N = info->len / 2; // I & Q per subcarrier
  if (N <= 0) return;
  ensure_buffers(N);

  // Compute amplitude |H_k| and phase φ_k = atan2(Q,I)
  // Also accumulate stats we need
  float sum_amp = 0.0f, sum_amp_sq = 0.0f;
  static float *amp = nullptr, *phi = nullptr;
  if (!amp) {
    amp = (float*)heap_caps_malloc(N*sizeof(float), MALLOC_CAP_8BIT);
    phi = (float*)heap_caps_malloc(N*sizeof(float), MALLOC_CAP_8BIT);
  }
  for (int k=0; k<N; ++k) {
    float I = (float)buf[2*k + 0];
    float Q = (float)buf[2*k + 1];
    float a = sqrtf(I*I + Q*Q);
    float p = atan2f(Q, I);
    amp[k] = a;
    phi[k] = p;
    sum_amp += a;
    sum_amp_sq += a*a;
  }

  // Unwrap φ across subcarriers (needed for delay from phase slope)
  unwrap_phase(phi, N);

  // ---------- FEATURE 2: Fading variation (per frame) ----------
  // across subcarriers: std/mean (coefficient of variation)
  float mean_amp = sum_amp / N;
  float var_amp  = max(0.0f, (sum_amp_sq / N) - mean_amp*mean_amp);
  float cv_sc    = (mean_amp > 1e-6f) ? sqrtf(var_amp)/mean_amp : 0.0f;

  // Temporal fading: update EMA + EW var per subcarrier (needs time series)
  // EWVar_t = (1-α)(EWVar_{t-1} + α (x_t - EMA_{t-1})^2)
  float ewvar_mean = 0.0f;
  for (int k=0;k<N;++k) {
    float prev_ema = g_amp_ema[k];
    float ema = (1-ALPHA)*prev_ema + ALPHA*amp[k];
    float ewvar = (1-ALPHA)*(g_amp_var[k] + ALPHA*sqf(amp[k] - prev_ema));
    g_amp_ema[k] = ema;
    g_amp_var[k] = ewvar;
    ewvar_mean += ewvar;
  }
  ewvar_mean /= N;  // average temporal variance across subcarriers

  // ---------- FEATURE 1 & 4: Scattering / material selectivity ----------
  // Use *spectral flatness measure* (SFM) and linear slope of amplitude vs subcarrier
  // SFM = exp(mean(log(a_k))) / mean(a_k). Lower SFM => more peaky/selective (scattering).
  float log_sum = 0.0f;
  for (int k=0;k<N;++k) log_sum += logf(max(amp[k], 1e-6f));
  float sfm = expf(log_sum / N) / max(mean_amp, 1e-6f);

  // Slope of amplitude (in dB) along subcarriers — material frequency response proxy
  static float *amp_dB = nullptr;
  if (!amp_dB) amp_dB = (float*)heap_caps_malloc(N*sizeof(float), MALLOC_CAP_8BIT);
  for (int k=0;k<N;++k) amp_dB[k] = 20.0f * log10f(max(amp[k], 1e-6f));
  float slope_amp_dB = 0.0f, intercept_amp_dB = 0.0f;
  linreg_y_vs_index(amp_dB, N, slope_amp_dB, intercept_amp_dB);
  // (units ≈ dB per subcarrier; multiply by subcarrier spacing for dB/Hz if needed)

  // ---------- FEATURE 6: Amplitude attenuation (per subcarrier vs baseline) ----------
  // After we capture BASELINE_FRAMES, baseline holds |H_k|_0. We report mean dB delta.
  float mean_delta_dB = 0.0f;
  if (!g_have_baseline) {
    // Accumulate baseline average
    for (int k=0;k<N;++k) g_amp_baseline[k] = (g_baseline_count == 0)
        ? amp[k]
        : ( (g_amp_baseline[k]*g_baseline_count + amp[k]) / (g_baseline_count+1) );
    g_baseline_count++;
    if (g_baseline_count >= BASELINE_FRAMES) g_have_baseline = true;
  } else {
    for (int k=0;k<N;++k) {
      float ratio = max(amp[k],1e-6f) / max(g_amp_baseline[k],1e-6f);
      mean_delta_dB += 20.0f * log10f(ratio);
    }
    mean_delta_dB /= N;
  }

  // ---------- FEATURE 5 & 7: Time delay & phase shift ----------
  // Group delay from phase-vs-frequency slope:
  //   φ_k ≈ φ0 - 2π f_k τ   =>  slope dφ/df ≈ -2π τ  => τ ≈ - (1/2π) dφ/df
  // Our x-axis is subcarrier index k; df = Δf = 312.5 kHz (20 MHz channel)
  const float delta_f = 312500.0f; // Hz
  float slope_phi = 0.0f, intercept_phi = 0.0f;
  linreg_y_vs_index(phi, N, slope_phi, intercept_phi);
  // convert slope per-subcarrier to per-Hz: (dφ/dk) / Δf
  float dphi_df = slope_phi / delta_f;        // rad/Hz
  float tau_s   = - (1.0f / (2.0f * (float)M_PI)) * dphi_df; // seconds
  float tau_ns  = tau_s * 1e9f; // convenient units

  // Phase shift vs baseline (fine sensing proxy)
  float mean_phase_delta = 0.0f;
  if (g_have_baseline) {
    // Create a quick phase baseline using current intercept as anchor
    // (If you also saved baseline phases, compare unwrap(φ-φ0))
    // Here we just remove linear trend to be robust:
    static float *phi_detr = nullptr;
    if (!phi_detr) phi_detr = (float*)heap_caps_malloc(N*sizeof(float), MALLOC_CAP_8BIT);
    for (int k=0;k<N;++k) phi_detr[k] = phi[k] - (intercept_phi + slope_phi * k);
    // Average residual phase magnitude
    float sum_abs = 0.0f;
    for (int k=0;k<N;++k) sum_abs += fabsf(phi_detr[k]);
    mean_phase_delta = sum_abs / N; // radians
  }

  // ---------- FEATURE 3: Distance (rough; needs calibration!) ----------
  // Log-distance model:  Pr(dBm) = A - 10 n log10(d)  =>  d = 10^((A - Pr)/10n)
  float d_m = powf(10.0f, (RX_REF_dBm - (float)rssi) / (10.0f * PATH_N));

  // ---------- PRINT CSV ----------
  // Columns:
  // ts_ms,rssi,chan,cv_sc,ewvar_mean,sfm,slope_amp_dB,mean_delta_dB,tau_ns,mean_phase_delta,d_est_m,mac
  uint32_t ts = millis();
  Serial.print(ts);                 Serial.print(',');
  Serial.print(rssi);               Serial.print(',');
  Serial.print(channel);            Serial.print(',');
  Serial.print(cv_sc, 6);           Serial.print(',');
  Serial.print(ewvar_mean, 6);      Serial.print(',');
  Serial.print(sfm, 6);             Serial.print(',');
  Serial.print(slope_amp_dB, 6);    Serial.print(',');
  Serial.print(g_have_baseline ? mean_delta_dB : NAN); Serial.print(',');
  Serial.print(tau_ns, 3);          Serial.print(',');
  Serial.print(mean_phase_delta, 6);Serial.print(',');
  Serial.print(d_m, 3);             Serial.print(',');
  print_mac(info->mac);
  Serial.println();

  // If you also want raw subcarrier features, uncomment below (verbose!):
  // Serial.print("#AMP,");
  // for (int k=0;k<N;++k){ Serial.print(amp[k],3); Serial.print(k+1<N?',':'\n'); }
  // Serial.print("#PHI,");
  // for (int k=0;k<N;++k){ Serial.print(phi[k],5); Serial.print(k+1<N?',':'\n'); }
}

static void setup_wifi_and_csi() {
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("Connecting to %s ...\n", WIFI_SSID);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) { delay(200); Serial.print('.'); }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connected. IP: "); Serial.println(WiFi.localIP());
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
      memcpy(g_connected_bssid, ap.bssid, 6);
      g_have_bssid = true;
      g_primary_channel = ap.primary;
      Serial.print("BSSID: "); print_mac(g_connected_bssid); Serial.println();
      Serial.print("Primary channel: "); Serial.println(g_primary_channel);
    }
  } else {
    Serial.println("[WARN] Not connected; still enabling CSI in monitor mode.");
  }

  wifi_promiscuous_filter_t filt = {};
  filt.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA;
  esp_wifi_set_promiscuous_filter(&filt);

  wifi_csi_config_t csi_config = {};
  csi_config.lltf_en           = true;
  csi_config.htltf_en          = true;
  csi_config.stbc_htltf2_en    = true;
  csi_config.ltf_merge_en      = true;
  csi_config.channel_filter_en = true;
  csi_config.manu_scale        = true;
  csi_config.shift             = 0;

  ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
  ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(csi_rx_cb, nullptr));
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
  ESP_ERROR_CHECK(esp_wifi_set_csi(true));

  Serial.println("CSI enabled. Capturing baseline for first frames...");
  Serial.println("CSV header: ts_ms,rssi,chan,cv_sc,ewvar_mean,sfm,slope_amp_dB,mean_delta_dB,tau_ns,mean_phase_delta,d_est_m,mac");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== ESP32 CSI Feature Extractor ===");
  setup_wifi_and_csi();
}
void loop() {
  static uint32_t last = 0;
  if (millis() - last > 5000) { last = millis(); Serial.println("# alive"); }
  delay(10);
}
