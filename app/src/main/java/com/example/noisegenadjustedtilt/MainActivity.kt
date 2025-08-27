package com.example.noisegenadjustedtilt


import android.media.AudioAttributes
import android.media.AudioFormat
import android.media.AudioTrack
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import kotlin.math.*
import kotlin.random.Random

/**
 * Вариант 2: спектральный синтез.
 *
 * На каждом шаге создаём комплексный спектр с маской:
 *   |X(f)| = tilt(f) * hp4(f), где tilt(f) = 10^(-4.5*log2(f/1k)/20)
 * Рандомные фазы → IFFT → Hann окно → 50% Overlap-Add (COLA).
 *
 * Без лимитера:
 *   masterGain = min( G_RMS, G_Peak ),
 *   где G_RMS подгоняет RMS под заданный dBFS по Parseval,
 *       G_Peak даёт жёсткий запас по пикам: max|y| < 0 dBFS даже при любой фазе.
 */
class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent { MaterialTheme { NoiseScreen() } }
    }
}

@Composable
fun NoiseScreen() {
    var running by remember { mutableStateOf(false) }
    var pan by remember { mutableFloatStateOf(0f) }              // –1..+1
    var decor by remember { mutableFloatStateOf(0f) }
    var tilt by remember { mutableFloatStateOf(-4.5f) }// 0..1
    var targetDbfs by remember { mutableFloatStateOf(-3.01f) }   // RMS синуса 0 dBFSpeak
    var isClamped by remember { mutableStateOf(false) }

    val engine = remember { IfftNoiseEngine() }

    LaunchedEffect(running) { if (running) engine.start() else engine.stop() }
    LaunchedEffect(pan) { engine.setPan(pan) }
    LaunchedEffect(decor) { engine.setDecorrelation(decor) }
    LaunchedEffect(tilt) { engine.setTilt(tilt) }
    LaunchedEffect(targetDbfs) { isClamped = engine.setTargetRmsDbfs(targetDbfs) }

    Surface(Modifier.fillMaxSize()) {
        Column(
            modifier = Modifier.fillMaxSize().padding(24.dp),
            verticalArrangement = Arrangement.spacedBy(24.dp, Alignment.CenterVertically),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Text("This app is a precise spectral-noise generator built with frequency-domain (IFFT) synthesis. Use it as a reference for live sound, audio mixing, room acoustics checks, gear testing, or simply for relaxation.", style = MaterialTheme.typography.titleMedium)

            Button(onClick = { running = !running }, modifier = Modifier.fillMaxWidth()) {
                Text(if (running) "Pause" else "Start")
            }

            Column(Modifier.fillMaxWidth()) {
                Text("Panning L/R: ${"%.2f".format(pan)}")
                Slider(value = pan, onValueChange = { pan = it.coerceIn(-1f, 1f) }, valueRange = -1f..1f)
            }

            Column(Modifier.fillMaxWidth()) {
                Text("Decorrelation L/R: ${"%.2f".format(decor)}")
                Slider(value = decor, onValueChange = { decor = it.coerceIn(0f, 1f) }, valueRange = 0f..1f)
            }

            Column(Modifier.fillMaxWidth()) {
                Text("Tilt: ${"%.2f".format(tilt)}")
                Slider(value = tilt, onValueChange = { tilt = it.coerceIn(-6f, 0f) }, valueRange = -6f..0f)
            }

            Column(Modifier.fillMaxWidth()) {
                val label = buildString {
                    append("Target RMS: "); append(String.format("%.1f dBFS", targetDbfs))
                    if (isClamped) append("  (gain is limited to not clip the signal)")
                }
                Text(label)
                Slider(
                    value = targetDbfs,
                    onValueChange = {
                        val v = (it.coerceIn(-60f, -3.0f) * 10f).roundToInt() / 10f
                        targetDbfs = v
                    },
                    valueRange = -60f..-3.0f
                )
            }

            Text("The generated noise spans 20 Hz–24 kHz, with a user-adjustable linear downward spectral tilt (0–6 dB per octave) and a 15 Hz high-pass filter.")
        }
    }
}

/** Реал-тайм генератор: IFFT → Hann окно → 50% overlap-add. */
/** Реал-тайм генератор: IFFT → √Hann окно → 50% overlap-add, с изменяемым tilt. */
class IfftNoiseEngine(
    private val sampleRate: Int = 48_000,
    private val bufferFrames: Int = 1024,
    private val N: Int = 8192
) {
    @Volatile private var running = false
    private var track: AudioTrack? = null
    private var thread: Thread? = null

    // управление
    @Volatile private var pan: Float = 0f
    @Volatile private var decor: Float = 0f
    @Volatile private var tilt: Float = -4.5f          // дБ/окт (относительно 1 кГц, отрицательный — спад)

    // целевой RMS (линейный) и гейн
    @Volatile private var targetRmsLin: Double = 10.0.pow(-3.01 / 20.0)
    @Volatile private var masterGain: Float = 1f
    @Volatile private var clamped: Boolean = false

    // «смелость» по пиковому запасу (как у тебя было): подбери по вкусу
    private var peakK: Double = 5.999

    // RNG
    private val rngL = Random(System.nanoTime())
    private val rngR = Random(System.nanoTime() xor 0x9E3779B97F4A7C15UL.toLong())

    // окно √Hann и OLA
    private val win = DoubleArray(N) { n -> sqrt(0.5 * (1.0 - cos(2.0 * Math.PI * n / N))) }
    private val hop = N / 2
    private val olaL = FloatArray(N)
    private val olaR = FloatArray(N)
    private var pos = 0
    private var samplesUntilNewBlock = 0

    // Спектральная маска (амплитуда, 0..Nyquist)
    private val mag = DoubleArray(N / 2 + 1)
    private val maskLock = Any()  // защита от гонок при перестройке маски

    // FFT буферы
    private val re = DoubleArray(N)
    private val im = DoubleArray(N)

    // Калибровки (должны пересчитываться при смене tilt)
    @Volatile private var rmsBase: Double = 1.0
    @Volatile private var peakBoundBase: Double = 1.0

    init {
        rebuildMaskAndCalibrations()  // первичная маска для стартового tilt

        recomputeGain()
    }

    fun setPan(value: Float) { pan = value; recomputeGain() }
    fun setDecorrelation(value: Float) { decor = value; recomputeGain() }

    /** Сменить наклон (дБ/окт) и сразу перестроить маску + откалибровать уровень. */
    fun setTilt(value: Float) {
        tilt = value
        rebuildMaskAndCalibrations()
        recomputeGain()
    }

    /** Установить целевой RMS (dBFS). */
    fun setTargetRmsDbfs(dbfs: Float): Boolean {
        targetRmsLin = 10.0.pow(dbfs / 20.0)
        return recomputeGain()
    }

    /** Пересчитать итоговый гейн из RMS-цели и пикового ограничения (учёт pan/decor). */
    private fun recomputeGain(): Boolean {
        val gRms = targetRmsLin / rmsBase

        // equal-power pan
        val theta = ((pan + 1f) * 0.5f) * (Math.PI / 2.0).toFloat()
        val gL = cos(theta).toDouble()
        val gR = sin(theta).toDouble()
        val panScale = max(gL, gR)  // [0.707 … 1.0]

        // учёт декорреляции (худший случай)
        val mixFactor = 0.5 + abs(0.5 - decor.toDouble()) // [0.5 … 1.0]

        val gPeak = peakK / (peakBoundBase * panScale * mixFactor)

        val g = min(gRms, gPeak)
        clamped = g < gRms - 1e-12
        masterGain = g.toFloat()
        return clamped
    }

    /** Верхняя безопасная граница RMS (для UI, если решишь ограничивать слайдер). */
    fun maxSafeRmsDbfs(): Float {
        val theta = ((pan + 1f) * 0.5f) * (Math.PI / 2.0).toFloat()
        val panScale = max(cos(theta).toDouble(), sin(theta).toDouble())
        val mixFactor = 0.5 + abs(0.5 - decor.toDouble())
        val gPeak = peakK / (peakBoundBase * panScale * mixFactor)
        val rmsMax = gPeak * rmsBase
        return (20.0 * ln(rmsMax.coerceAtLeast(1e-20)) / ln(10.0)).toFloat()
    }

    fun start() {
        if (running) return
        running = true

        val minBuf = AudioTrack.getMinBufferSize(
            sampleRate,
            AudioFormat.CHANNEL_OUT_STEREO,
            AudioFormat.ENCODING_PCM_16BIT
        )
        val frameBytes = 4
        val bufBytes = max(minBuf, bufferFrames * frameBytes)

        track = AudioTrack.Builder()
            .setAudioAttributes(
                AudioAttributes.Builder()
                    .setUsage(AudioAttributes.USAGE_MEDIA)
                    .setContentType(AudioAttributes.CONTENT_TYPE_MUSIC)
                    .build()
            )
            .setAudioFormat(
                AudioFormat.Builder()
                    .setEncoding(AudioFormat.ENCODING_PCM_16BIT)
                    .setSampleRate(sampleRate)
                    .setChannelMask(AudioFormat.CHANNEL_OUT_STEREO)
                    .build()
            )
            .setTransferMode(AudioTrack.MODE_STREAM)
            .setBufferSizeInBytes(bufBytes)
            .build()
            .apply { play() }

        java.util.Arrays.fill(olaL, 0f)
        java.util.Arrays.fill(olaR, 0f)
        pos = 0
        samplesUntilNewBlock = 0

        addNewBlock(rngL, olaL)
        addNewBlock(rngR, olaR)

        thread = Thread(this::renderLoop, "NoiseIFFT").also { it.start() }
    }

    fun stop() {
        running = false
        thread?.join(300)
        thread = null
        track?.let { t -> try { t.stop() } catch (_: Throwable) {}; try { t.release() } catch (_: Throwable) {} }
        track = null
    }

    private fun renderLoop() {
        val t = track ?: return
        val out = ShortArray(bufferFrames * 2)

        while (running) {
            var produced = 0
            while (produced < bufferFrames) {
                if (samplesUntilNewBlock <= 0) {
                    addNewBlock(rngL, olaL)
                    addNewBlock(rngR, olaR)
                    samplesUntilNewBlock = hop
                }

                val chunk = min(samplesUntilNewBlock, bufferFrames - produced)

                // снимки параметров на чанк
                val mg = masterGain
                val theta = ((pan + 1f) * 0.5f) * (Math.PI / 2.0).toFloat()
                val gL = cos(theta); val gR = sin(theta)
                val dec = decor
                val kCommon = (1f - dec) * 0.5f
                val kDiff   = dec * 0.5f

                for (i in 0 until chunk) {
                    val l0 = olaL[pos] * mg
                    val r0 = olaR[pos] * mg
                    olaL[pos] = 0f; olaR[pos] = 0f

                    val common = kCommon * (l0 + r0)
                    val diff   = kDiff   * (l0 - r0)
                    var yl = (common + diff) * gL
                    var yr = (common - diff) * gR

                    out[(produced + i) * 2]     = (yl * 32767f).toInt().toShort()
                    out[(produced + i) * 2 + 1] = (yr * 32767f).toInt().toShort()

                    pos++; if (pos >= N) pos = 0
                }

                produced += chunk
                samplesUntilNewBlock -= chunk
            }

            val wrote = t.write(out, 0, out.size, AudioTrack.WRITE_BLOCKING)
            if (wrote < 0) break
        }
    }

    // ===== генерация и OLA =====
    private fun addNewBlock(rng: Random, ola: FloatArray) {
        val half = N / 2
        synchronized(maskLock) {
            for (k in 1 until half) {
                val phi = 2.0 * Math.PI * rng.nextDouble()
                val m = mag[k]
                re[k] = m * cos(phi)
                im[k] = m * sin(phi)
                // гермитова симметрия
                re[N - k] = re[k]
                im[N - k] = -im[k]
            }
            // DC≈0 после HP, Nyquist — действительный
            re[0] = 0.0; im[0] = 0.0
            re[half] = mag[half]; im[half] = 0.0
        }

        ifft(re, im)

        var p = pos
        for (n in 0 until N) {
            val v = (re[n] * win[n]).toFloat()
            val j = p + n
            val idx = if (j < N) j else j - N
            ola[idx] += v
        }
    }

    // ===== перестройка маски и калибровок при смене tilt =====
    private fun rebuildMaskAndCalibrations() {
        val fs = sampleRate.toDouble()
        synchronized(maskLock) {
            // 1) Маска |X(f)| = tilt * hp4, новая tilt из поля this.tilt
            for (k in 0..N/2) {
                val f = k * fs / N
                mag[k] = targetAmp(f)
            }
            // 2) Нормировка на 0 дБ @ 1 кГц
            val kRef = ((1000.0 / fs) * N).roundToInt().coerceIn(0, N/2)
            val ref = mag[kRef].takeIf { it > 0.0 } ?: 1.0
            val s = 1.0 / ref
            for (k in 0..N/2) mag[k] *= s

            // 3) Пересчёт базовых RMS и пикового лимита
            var sumSq = 0.0
            var sumAbs = 0.0
            for (k in 1 until N/2) {
                val m = mag[k]
                sumSq += 2.0 * m * m
                sumAbs += 2.0 * m
            }
            val m0 = mag[0]
            val mN = mag[N/2]
            sumSq += m0*m0 + mN*mN
            sumAbs += m0 + mN

            val rmsBlock = sqrt(sumSq) / N.toDouble()
            val avgWinSq = averageWinSqWithOverlap()   // для √Hann+50% ≈ 1.0
            rmsBase = rmsBlock * sqrt(avgWinSq)

            val peakWinSum = peakWinSumWithOverlap()   // максимум суммы перекрытых окон
            peakBoundBase = (sumAbs / N.toDouble()) * peakWinSum
        }
    }

    // ===== расчёты окна/перекрытия =====
    private fun averageWinSqWithOverlap(): Double {
        var s = 0.0
        for (n in 0 until N) {
            val a = win[n]
            val b = win[(n + hop) % N]
            s += a*a + b*b
        }
        return s / N
    }
    private fun peakWinSumWithOverlap(): Double {
        var m = 0.0
        for (n in 0 until N) {
            val v = win[n] + win[(n + hop) % N]
            if (v > m) m = v
        }
        return m
    }

    // ===== целевая амплитуда =====
    private fun targetAmp(f: Double): Double {
        if (f <= 0.0) return 0.0
        val tiltDb = (tilt) * (ln(f / 1000.0) / ln(2.0))  // дБ/окт относительно 1 кГц
        val tiltLin = 10.0.pow(tiltDb / 20.0)
        val hp = hp4Mag(f, 15.0)
        return tiltLin * hp
    }
    private fun hp4Mag(f: Double, fc: Double): Double {
        if (f <= 0.0) return 0.0
        val r = fc / f
        return 1.0 / sqrt(1.0 + r.pow(8.0))  // Баттерворт N=4 по модулю
    }

    // ===== FFT/IFTT =====
    private fun fft(re: DoubleArray, im: DoubleArray, inverse: Boolean) {
        val n = re.size
        var j = 0
        for (i in 1 until n) {
            var bit = n shr 1
            while (j and bit != 0) { j = j xor bit; bit = bit shr 1 }
            j = j xor bit
            if (i < j) {
                val tr = re[i]; re[i] = re[j]; re[j] = tr
                val ti = im[i]; im[i] = im[j]; im[j] = ti
            }
        }
        var len = 2
        while (len <= n) {
            val ang = 2.0 * Math.PI / len * if (inverse) -1 else 1
            val wlenRe = cos(ang)
            val wlenIm = sin(ang)
            for (i in 0 until n step len) {
                var wRe = 1.0
                var wIm = 0.0
                val half = len / 2
                for (k in 0 until half) {
                    val uRe = re[i + k]
                    val uIm = im[i + k]
                    val vRe = re[i + k + half] * wRe - im[i + k + half] * wIm
                    val vIm = re[i + k + half] * wIm + im[i + k + half] * wRe
                    re[i + k] = uRe + vRe
                    im[i + k] = uIm + vIm
                    re[i + k + half] = uRe - vRe
                    im[i + k + half] = uIm - vIm
                    val nwRe = wRe * wlenRe - wIm * wlenIm
                    val nwIm = wRe * wlenIm + wIm * wlenRe
                    wRe = nwRe; wIm = nwIm
                }
            }
            len = len shl 1   // ← эта строка была пропущена
        }
        if (inverse) {
            for (i in 0 until n) { re[i] /= n; im[i] /= n }
        }
    }

    private fun ifft(re: DoubleArray, im: DoubleArray) = fft(re, im, true)
}

