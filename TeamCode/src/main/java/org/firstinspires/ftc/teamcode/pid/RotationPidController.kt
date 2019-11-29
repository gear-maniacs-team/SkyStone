package org.firstinspires.ftc.teamcode.pid

import com.qualcomm.robotcore.util.Range
import kotlin.math.abs

class RotationPidController(
    Kp: Double,
    Ki: Double,
    Kd: Double
) : PidController(Kp, Ki, Kd) {

    private var previousTime = 0L
    private var cumulativeError = 0.0
    private var lastError = 0.0
    private var lastOutput = 0.0
    private var actualOutput = 0.0

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
        val direction = if (inputBounded) {
            val average = (minInput + maxInput) / 2
            if (average - input <= average + input) 1.0 else -1.0
        } else 1.0

        val clippedInput = Range.clip(input, minInput, maxInput)
        val error = direction * abs(target - clippedInput)

        val currentTime = System.currentTimeMillis()
        val deltaTime = currentTime - previousTime

        if (lastOutput > minOutput && lastOutput < maxOutput)
            cumulativeError += error * deltaTime

        val derivative = (error - lastError) / deltaTime

        var output = Kp * error + Ki * cumulativeError * direction + Kd * derivative

        lastError = error
        previousTime = currentTime

        actualOutput = output
        if (outputBounded)
            output = Range.clip(output, minOutput, maxOutput)

        lastOutput = output
        return output
    }

    override fun toString(): String {
        return super.toString() + " Cumulative=$cumulativeError Output=$actualOutput"
    }
}