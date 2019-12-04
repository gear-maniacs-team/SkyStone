package org.firstinspires.ftc.teamcode.pid

import android.util.Log
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.utils.Ranges
import kotlin.math.sign

class GeneralPidController(
    Kp: Double,
    Ki: Double,
    Kd: Double
) : AbstractPidController(Kp, Ki, Kd) {

    private var firstRun = true
    private var previousTime = 0L
    private var cumulativeError = 0.0
    private var lastError = 0.0
    private var lastOutput = 0.0

    var debugEnabled = false
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

        val clippedInput = if (inputBounded) Range.clip(input, minInput, maxInput) else input
        val error = target - clippedInput

        val currentTime = System.currentTimeMillis()
        val deltaTime = currentTime - previousTime

        if (Ranges.isRangeValid(lastOutput, minOutput, maxOutput) || sign(cumulativeError) != sign(error))
            cumulativeError += error * deltaTime

        val derivative = (error - lastError) / deltaTime

        val output = Kp * error + Ki * cumulativeError + Kd * derivative
        lastOutput = output

        lastError = error
        previousTime = currentTime

        val clippedOutput = if (outputBounded) Range.clip(output, minOutput, maxOutput) else output

        if (debugEnabled) {
            Log.d(
                "PID",
                "Input=$input Error=$error CumulativeError=$cumulativeError Output=$output ClippedOutput=$output"
            )
        }

        return clippedOutput
    }
}
