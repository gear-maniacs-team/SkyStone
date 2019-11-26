package org.firstinspires.ftc.teamcode.pid

import com.qualcomm.robotcore.util.Range

class RotationPidController(
    Kp: Double,
    Ki: Double,
    Kd: Double
) : PidController(Kp, Ki, Kd) {

    private var previousTime = 0L
    private var cumulativeError = 0.0
    private var lastError = 0.0

    var inputBounded = false
    var outputBounded = false
    var minInput = 0.0
        private set
    var maxInput = 0.0
        private set
    var minOutput = 0.0
        private set
    var maxOutput = 0.0
        private set
    var maxError = 0.0

    fun setInputRange(min: Double, max: Double) {
        require(min < max)

        inputBounded = true
        minInput = min
        maxInput = max
    }

    fun setOutputRange(min: Double, max: Double) {
        require(min < max)

        outputBounded = true
        minOutput = min
        maxOutput = max
    }

    override fun compute(input: Double): Double {
        val error = target - Range.clip(input, minInput, maxInput)

        val currentTime = System.currentTimeMillis()
        val deltaTime = currentTime - previousTime

        cumulativeError += error * deltaTime
        if (cumulativeError > maxError)
            cumulativeError = maxError

        val derivative = (error - lastError) / deltaTime

        var output = Kp * error + Ki * cumulativeError + Kd * derivative

        lastError = error
        previousTime = currentTime

        if (inputBounded) {
            val average = (minInput + maxInput) / 2
            val sign = if (average - input <= average + input) 1.0 else -1.0

            output *= sign
        }

        if (outputBounded)
            output = Range.clip(output, minOutput, maxOutput)

        return output
    }
}