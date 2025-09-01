package com.example.noisegenadjustedtilt

import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.lifecycle.SavedStateHandle
import androidx.lifecycle.ViewModel

/**
 * Single source of truth for UI + audio engine.
 * Keeps one engine instance across configuration changes.
 * Persists user parameters via SavedStateHandle to recover after process death.
 * Does NOT auto-stop in background; user controls Start/Pause explicitly.
 */
class NoiseViewModel(
    private val state: SavedStateHandle
) : ViewModel() {

    // One engine instance for the whole screen lifecycle
    val engine = IfftNoiseEngine()

    // UI state (persisted)
    var running    by mutableStateOf(state["running"]    ?: false);    private set
    var pan        by mutableStateOf(state["pan"]        ?: 0f);       private set
    var decor      by mutableStateOf(state["decor"]      ?: 0f);       private set
    var tilt       by mutableStateOf(state["tilt"]       ?: -4.5f);    private set
    var targetDbfs by mutableStateOf(state["targetDbfs"] ?: -3.01f);   private set
    var isClamped  by mutableStateOf(false);                            private set

    init {
        // Re-apply persisted parameters to the engine
        engine.setPan(pan)
        engine.setDecorrelation(decor)
        engine.setTilt(tilt)
        isClamped = engine.setTargetRmsDbfs(targetDbfs)
        if (running) engine.start()
    }

    fun toggleRunning() {
        val newVal = !running
        running = newVal
        state["running"] = newVal
        if (newVal) engine.start() else engine.stop()
    }

    // --- Renamed methods to avoid JVM signature clash with property setters ---

    fun updatePan(v: Float) {
        val x = v.coerceIn(-1f, 1f)
        pan = x; state["pan"] = x
        engine.setPan(x)
        isClamped = engine.setTargetRmsDbfs(targetDbfs)
    }

    fun updateDecor(v: Float) {
        val x = v.coerceIn(0f, 1f)
        decor = x; state["decor"] = x
        engine.setDecorrelation(x)
        isClamped = engine.setTargetRmsDbfs(targetDbfs)
    }

    fun updateTilt(v: Float) {
        val x = v.coerceIn(-6f, 0f)
        tilt = x; state["tilt"] = x
        engine.setTilt(x)
        isClamped = engine.setTargetRmsDbfs(targetDbfs)
    }

    fun updateTargetDbfs(dbfs: Float) {
        val x = dbfs.coerceIn(-60f, -3.0f)
        targetDbfs = x; state["targetDbfs"] = x
        isClamped = engine.setTargetRmsDbfs(x)
    }

    override fun onCleared() {
        // No leaked audio thread when the screen/process truly goes away
        engine.stop()
    }
}
