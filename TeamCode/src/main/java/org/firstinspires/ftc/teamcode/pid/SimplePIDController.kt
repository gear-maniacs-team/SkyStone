package org.firstinspires.ftc.teamcode.pid

class SimplePIDController(
    Kp: Double,
    Ki: Double,
    Kd: Double
) : PidController(Kp, Ki, Kd) {

    private var currentTime: Long = 0
    private var previousTime: Long = 0
    private var elapsedTime: Long = 0
    private var error: Double = 0.0
    private var cumulativeError: Double = 0.0
    private var errorRate: Double = 0.0
    private var lastError: Double = 0.0

    override fun computePID(input: Double): Double {
        currentTime = System.currentTimeMillis()
        elapsedTime = (currentTime - previousTime)

        error = target - input
        cumulativeError += error * elapsedTime
        errorRate = (error - lastError) / elapsedTime

        val output = Kp * error + Ki * cumulativeError + Kd * errorRate

        lastError = error
        previousTime = currentTime

        return output
    }
}
