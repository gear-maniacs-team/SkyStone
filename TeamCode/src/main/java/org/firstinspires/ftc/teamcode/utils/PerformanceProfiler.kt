package org.firstinspires.ftc.teamcode.utils

import org.firstinspires.ftc.robotcore.external.Telemetry

class PerformanceProfiler {

    private var firstRun = true
    private var oldTime = 0L
    private var framesCount = 0
    private var result = 0.0

    private var lastTelemetryItem: Telemetry.Item? = null

    /*
     * Returns milliseconds / updates
     */
    fun update(): Double {
        val currentTime = System.currentTimeMillis()

        if (firstRun) {
            oldTime = currentTime
            firstRun = false
            return 0.0
        }

        framesCount++
        if (currentTime - oldTime >= 1000) {
            result = 1000.0 / framesCount
            framesCount = 0
            oldTime += 1000
        }

        return result
    }

    fun update(telemetry: Telemetry, caption: String = "Ms/Updates") {
        val ms = update()

        lastTelemetryItem?.let {
            telemetry.removeItem(it)
        }
        lastTelemetryItem = telemetry.addData(caption, ms)
    }
}
