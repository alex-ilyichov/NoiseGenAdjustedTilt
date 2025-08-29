# NoiseGen — Adjustable Spectral Noise (Android)

NoiseGen generates noise via **frequency-domain (IFFT) synthesis** with an adjustable **linear spectral tilt (0–6 dB/oct)** and a **15 Hz high-pass**.  
Use it as a reference for live sound, mixing, room checks, gear testing, or just for relaxation.

### Features
- **Tilt**: 0–6 dB per octave (downward slope), normalized at 1 kHz
- **15 Hz high-pass** (≈ –3 dB @ 15 Hz)
- **Target RMS (dBFS)** with **peak-safe** gain (no limiter; calibrated by Parseval)
- **Pan** (equal-power) and **L/R decorrelation** controls
- 48 kHz, 16-bit stereo output
- No ads, no networking, no data collection

### How it works (DSP)
- Spectral synthesis: build magnitude mask `|X(f)| = tilt(f) * hp4(f)`,
  random phases, **IFFT → √Hann window → 50% overlap-add** (COLA by energy).
- Tilt is referenced to 1 kHz; HP is a 4th-order Butterworth-like magnitude.
- **Gain calibration**:
    - `G_RMS` chosen to hit the requested RMS (Parseval-based base RMS).
    - `G_Peak` provides headroom so inter-sample peaks do not exceed 0 dBFS,  
      also accounting for pan/decor worst case.
    - Final gain = `min(G_RMS, G_Peak)` → **no limiter needed**.

### Controls
- **Start/Pause**
- **Pan** (–1 … +1, equal-power)
- **Decorrelation** (0 … 1: from mono to fully de-correlated)
- **Tilt** (0 … –6 dB/oct)
- **Target RMS** (dBFS)

### Build
```bash
# Debug APK
./gradlew assembleDebug
# app/build/outputs/apk/debug/app-debug.apk

# Release (unsigned by default; configure signing if needed)
./gradlew assembleRelease
