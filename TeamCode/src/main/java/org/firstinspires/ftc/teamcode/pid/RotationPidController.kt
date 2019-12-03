package org.firstinspires.ftc.teamcode.pid

import android.util.Log
import com.qualcomm.robotcore.util.Range
import kotlin.math.abs
import kotlin.math.sign

class RotationPidController(
    Kp: Double,
    Ki: Double,
    Kd: Double,
    Kf: Double
) : PidController(Kp, Ki, Kd) {

    private var firstRun = true
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
        if (firstRun) {
            firstRun = false
            previousTime = System.currentTimeMillis()
            cumulativeError = 0.0
        }

        val clippedInput = Range.clip(input, minInput, maxInput)
        val error = target - clippedInput

        val currentTime = System.currentTimeMillis()
        val deltaTime = currentTime - previousTime

        if ((lastOutput > minOutput && lastOutput < maxOutput) || (sign(cumulativeError) != sign(error)))
            cumulativeError += error * deltaTime

        val derivative = (error - lastError) / deltaTime

        var output = Kp * error + Ki * cumulativeError + Kd * derivative

        lastError = error
        previousTime = currentTime

        actualOutput = output
        if (outputBounded)
            output = Range.clip(output, minOutput, maxOutput)

        lastOutput = output

        Log.d("PID", "Input=$input Error=$error CumulativeError=$cumulativeError ActualOutput=$actualOutput Output=$output")

        return output
    }
}
