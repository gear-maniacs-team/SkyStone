package org.firstinspires.ftc.teamcode.utils

import org.firstinspires.ftc.robotcore.external.Telemetry

class UPSCounter {

    private var firstRun = true
    private var lastTime = 0L
    private var updatesCount = 0
    private var lastUpdateRate = 0

    private var lastTelemetryItem: Telemetry.Item? = null

    fun update(): Int {
        val currentTime = System.currentTimeMillis()
        ++updatesCount

        if (firstRun) {
            lastTime = currentTime
        }

        val deltaTime = currentTime - lastTime
        if (deltaTime > 1000) { // if more than one second passed
            lastUpdateRate = updatesCount
            updatesCount = 0
            lastTime = currentTime
        }

        return lastUpdateRate
    }

    fun update(telemetry: Telemetry, caption: String = "UPS") {
        val rate = update()

        lastTelemetryItem?.let {
            telemetry.removeItem(it)
        }
        lastTelemetryItem = telemetry.addData(caption, rate)
    }
}
