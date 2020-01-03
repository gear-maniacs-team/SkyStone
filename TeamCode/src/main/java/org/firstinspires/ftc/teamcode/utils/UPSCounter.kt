package org.firstinspires.ftc.teamcode.utils

import org.firstinspires.ftc.robotcore.external.Telemetry

class UPSCounter {

    private var firstRun = true
    private var oldTime = 0L

    private var lastTelemetryItem: Telemetry.Item? = null

    fun update(): Double {
        val currentTime = System.nanoTime()

        if (firstRun) {
            oldTime = currentTime
            return 0.0
        }

        val delta = currentTime - oldTime
        val ups: Double = 1.0 / (delta * 1000)
        oldTime = currentTime

        return ups
    }

    fun update(telemetry: Telemetry, caption: String = "UPS") {
        val rate = update()

        lastTelemetryItem?.let {
            telemetry.removeItem(it)
        }
        lastTelemetryItem = telemetry.addData(caption, "%2.0f", rate)
    }
}
