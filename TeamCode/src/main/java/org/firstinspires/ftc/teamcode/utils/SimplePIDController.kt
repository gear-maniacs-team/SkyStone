package org.firstinspires.ftc.teamcode.utils

import org.firstinspires.ftc.robotcore.internal.system.Misc

class SimplePIDController(
    var kp: Double,
    var ki: Double,
    var kd: Double
) {

    var target: Double = 0.0

    private var currentTime: Long = 0
    private var previousTime: Long = 0
    private var elapsedTime: Long = 0
    private var error: Double = 0.0
    private var cumulativeError: Double = 0.0
    private var errorRate: Double = 0.0
    private var lastError: Double = 0.0

    fun computePID(input: Double): Double {
        currentTime = System.currentTimeMillis()
        elapsedTime = (currentTime - previousTime)

        error = target - input
        cumulativeError += error * elapsedTime
        errorRate = (error - lastError) / elapsedTime

        val output = kp * error + ki * cumulativeError + kd * errorRate

        lastError = error
        previousTime = currentTime

        return output
    }

    override fun toString(): String = Misc.formatForUser("p=%f i=%f d=%f", kp, ki, kd)
}
