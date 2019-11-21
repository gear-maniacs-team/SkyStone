package org.firstinspires.ftc.teamcode.utils

class SimplePIDController {
    private val kp = 0
    private val ki = 0
    private val kd = 0
    var setPoint: Double = 0.0
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

        error = setPoint - input
        cumulativeError += error * elapsedTime
        errorRate = (error - lastError) / elapsedTime

        val output = kp * error + ki * cumulativeError + kd * errorRate

        lastError = error
        previousTime = currentTime

        return output
    }


}