package org.firstinspires.ftc.teamcode.utils

import org.firstinspires.ftc.robotcore.external.Telemetry

class UPSCounter {

    private var firstRun = true
    private var lastTime = 0L
    private var updatesCount = 0
    private var updateRate = 30f

    private var lastTelemetryItem: Telemetry.Item? = null

    fun update(): Float {
        val currentTime = System.currentTimeMillis()
        ++updatesCount

        if (firstRun) {
            lastTime = currentTime
        }

        val deltaTime = currentTime - lastTime
        if (deltaTime > 1000) { // if more than one second passed
            updateRate = updatesCount * 0.5f + updateRate * 0.5f
            updatesCount = 0
            lastTime = currentTime
        }

        return updateRate
    }

    fun update(telemetry: Telemetry) {
        val rate = update()

        lastTelemetryItem?.let {
            telemetry.removeItem(it)
        }
        lastTelemetryItem = telemetry.addData("UPS", rate)
    }
}
