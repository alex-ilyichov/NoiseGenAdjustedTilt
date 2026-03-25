# NoiseGen — Adjustable Spectral Noise (Android)

A precision noise generator for Android built around frequency-domain synthesis. Designed for live sound reference, room acoustics checks, audio gear testing, and mixing work where a calibrated, spectrally shaped noise source is needed.

No ads, no networking, no data collection.

---

## Why this exists

When you mix in a room, you're always hearing two things at once: your mix and your room. Even accurate monitors accumulate low-mid energy from early reflections and room modes. The diffuse field tilts toward the low end. Measurements of world-class classical concert halls show the same pattern — more low-frequency energy relative to highs in the reverberant field. Your room does something similar, at a smaller scale.

This creates a practical problem. If your room is loading up the low end, you'll compensate by reducing low frequencies in your mix. Then the mix leaves your room and sounds thin everywhere else.

Academic research on commercial masters from 1950–2010 ([Pestana et al.](https://www.researchgate.net/publication/274511175_Spectral_characteristics_of_popular_commercial_recordings_1950-2010)) measured a consistent spectral tilt of approximately **4.5–5 dB/oct** in the 100–4000 Hz range — steeper than pink noise (–3 dB/oct), meaning more energy in the lows relative to the highs. More recent measurement practice and empirical listening tests suggest the current tendency is closer to **4 dB/oct** across the full audible range, reflecting the spectral flattening trend in modern masters.

Pink noise at –3 dB/oct is actually **brighter** than commercial masters — it loses less energy per octave going up in frequency. If your room sounds balanced to you at –3 dB/oct, your monitoring environment is brighter than the references you're comparing against.

A spectrally adjustable noise generator helps in two ways:

**Monitoring calibration:** Find the tilt setting that makes your room feel balanced. Run commercial reference mixes through the same compensating EQ curve. Learn how professionally mastered material sounds in your environment. Mix through that corrected chain. When rendering the final mix for mastering, disable the compensating EQ — the mastering engineer receives a mix made in a calibrated environment. Less corrective work needed at the mastering stage.

**Communication with your mastering engineer:** Tell them the tilt you were monitoring with — for example, "I mixed using a +1 dB/oct brightening pivot EQ at 1 kHz to compensate for my room." That's a precise spectral reference point for your decisions. A shared technical language instead of subjective descriptions like "a bit dark" or "slightly bright."

Both use cases depend on the same requirement: a clean, stable, spectrally precise reference signal that doesn't color the measurement.

---

## The problem with simple noise generators

Most noise generator apps produce white noise in the time domain — generate a random sample, output it. That gives you no spectral control. The other problem is clipping: increase the gain on a naive generator and inter-sample peaks go over 0 dBFS, introducing distortion that contaminates exactly the signal you're using as a reference.

The standard fix — a limiter or compressor — has the same problem. It modifies the spectrum dynamically. For a reference signal that's unacceptable.

The requirement was: gain up without clipping, no matter what pan or decorrelation settings are active. No limiter. No compressor. No spectral surprises.

---

## Frequency-domain synthesis

NoiseGen builds the noise in the frequency domain and transforms it into audio, rather than generating samples and filtering them:

```
1. Build magnitude spectrum:  |X(f)| = tilt(f) × hp4(f)
2. Assign random phases:      φ_k ~ Uniform[0, 2π)
3. Enforce Hermitian symmetry for real output
4. IFFT → time-domain block
5. Apply √Hann window
6. Overlap-add at 50% hop
7. Output to AudioTrack
```

**Tilt** shapes the spectrum as a function of frequency, referenced to 1 kHz:

- 0 dB/oct → white noise (flat spectrum)
- –3 dB/oct → pink noise (equal energy per octave; brighter than commercial masters)
- –6 dB/oct → red/Brownian noise

The 1 kHz reference means the level at 1 kHz stays constant as you adjust tilt — you're reshaping the spectrum, not shifting the overall level.

**High-pass** at 15 Hz removes infrasonic content that wastes headroom without contributing anything audible.

---

## The windowing problem

IFFT synthesis produces discrete blocks of samples. Concatenating them naively creates audible glitches at the boundaries. The fix is overlap-add: each block overlaps the previous by 50% and the overlapping regions are summed.

For this to produce a smooth output, the window function must satisfy the **Constant Overlap-Add (COLA)** property — the summed energy at every sample must equal 1. A standard Hann window doesn't satisfy this in the synthesis context. The solution is the **square-root Hann window**, which guarantees COLA compliance at 50% overlap with no boundary artifacts.

---

## Gain calibration without a limiter

The gain calculation uses two values computed analytically before any audio plays:

**G_RMS** targets the requested output level using the energy relationship between the frequency and time domains ([Parseval's theorem](https://en.wikipedia.org/wiki/Parseval%27s_theorem)). Computed once per tilt change — stable and jitter-free.

**G_Peak** analytically bounds the worst-case instantaneous peak across all combinations of pan and decorrelation, providing enough headroom that peaks never reach 0 dBFS regardless of parameter settings.

Final gain: `min(G_RMS, G_Peak)`.

When the peak constraint is tighter than the RMS target, the UI flags it — "(gain limited by peak guard)". Lowering the RMS target recovers full dynamic range. The audio stays clean either way.

This is why pushing the gain beyond a certain point produces diminishing returns in perceived loudness — the peak guard is holding the ceiling. That's intentional behavior, not a bug.

---

## Stereo: pan and decorrelation

Two independent noise streams are generated per block.

**Pan** uses equal-power panning — total energy stays constant as you pan, which is correct for a reference signal.

**Decorrelation** controls channel independence:

- `0` → mono: both channels carry the same signal
- `1` → fully independent: L and R are uncorrelated noise streams

The peak gain calculation accounts for worst-case amplitude across all pan and decorrelation combinations — the no-clipping guarantee holds everywhere in the parameter space.

---

## Lifecycle and process-death survival

Android can kill the app process at any time when backgrounded. All parameters — running state, pan, decorrelation, tilt, and target RMS — are persisted via `SavedStateHandle` in the ViewModel. On process recreation, the UI restores to its exact prior state. If audio was playing when the process was killed, it resumes automatically.

Playback intentionally does not pause when the app is backgrounded. The use case is prolonged monitoring — PA tuning, room measurement sessions, gear testing — where the phone screen may be off.

---

## Build

Requires [Android Studio](https://developer.android.com/studio) or the Android SDK command-line tools. Min SDK 31 (Android 12).

```bash
# Debug APK
./gradlew assembleDebug
# Output: app/build/outputs/apk/debug/app-debug.apk

# Release (configure signing in build.gradle.kts before release builds)
./gradlew assembleRelease
```

---

## Controls

| Control | Range | Description |
|---|---|---|
| Start / Pause | — | Starts or stops audio output |
| Tilt | 0 to –6 dB/oct | Spectral slope; 0 = white, –3 = pink, –6 = red |
| Pan | –1 to +1 | Equal-power stereo pan |
| Decorrelation | 0 to 1 | 0 = mono, 1 = fully independent L/R |
| Target RMS | –60 to –3 dBFS | Output level; peak guard activates if RMS target would clip |

---

## Technical specifications

| Parameter | Value |
|---|---|
| Sample rate | 48 kHz |
| Bit depth | 16-bit signed PCM |
| Channels | Stereo |
| FFT block size | 8192 samples |
| Hop size | 4096 samples (50% overlap) |
| Frequency range | 20 Hz – 24 kHz |
| High-pass corner | 15 Hz (4th-order Butterworth-like) |
| Tilt reference | 1 kHz (0 dB) |

---

## Known constraints

| Constraint | Detail |
|---|---|
| FFT block size | Fixed at N=8192. No dynamic adjustment. |
| No audio focus handling | Playback does not pause on notification or call interruption. |
| No output recording | Audio plays to speaker/headphone output only; no file export. |
| 16-bit output | Sufficient for noise reference use; not suitable for mastering-grade work. |
| Min SDK 31 | Android 12 required. |

---

## License

See [LICENSE.md](LICENSE.md).
