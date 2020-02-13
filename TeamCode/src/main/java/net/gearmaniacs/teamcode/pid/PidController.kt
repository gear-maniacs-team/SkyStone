package net.gearmaniacs.teamcode.pid

import android.util.Log
import com.qualcomm.robotcore.util.Range
import net.gearmaniacs.teamcode.utils.Ranges
import net.gearmaniacs.teamcode.utils.RobotClock
import net.gearmaniacs.teamcode.utils.extensions.epsilonEquals
import org.firstinspires.ftc.robotcore.internal.system.Misc
import kotlin.math.abs
import kotlin.math.sign

class PidController(
    var Kp: Double,
    var Ki: Double,
    var Kd: Double
) {

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

    var setPoint: Double = 0.0

    fun reset() {
        previousTime = RobotClock.millis()
        cumulativeError = 0.0
        lastError = 0.0
        lastOutput = 0.0
    }

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

    private fun getError(input: Double): Double {
        var error = setPoint - input

        if (inputBounded) {
            val inputRange = maxInput - minInput

            while (abs(error) > inputRange / 2.0)
                error -= sign(error) * inputRange
        }

        return error
    }

    fun compute(input: Double): Double {
        val error = getError(input)

        if (firstRun) {
            firstRun = false

            lastError = error
            previousTime = RobotClock.millis()
            return 0.0
        }

        val currentTime = RobotClock.millis()
        val deltaTime = currentTime - previousTime
        if (deltaTime == 0L)
            return 0.0

        if (Ranges.isRangeValid(lastOutput, minOutput, maxOutput) || sign(cumulativeError) != sign(error))
            cumulativeError += error * deltaTime

        val derivative = (error - lastError) / deltaTime

        val baseOutput = Kp * error + Ki * cumulativeError + Kd * derivative
        val output = if (baseOutput epsilonEquals 0.0) 0.0 else baseOutput

        lastOutput = output
        lastError = error
        previousTime = currentTime

        if (debugEnabled) {
            Log.d(
                "PID Debug",
                "Input=$input Error=$error CumulativeError=$cumulativeError Output=$output"
            )
        }

        return if (outputBounded) Range.clip(output, minOutput, maxOutput) else output
    }

    override fun toString(): String = Misc.formatForUser("P=%f I=%f D=%f", Kp, Ki, Kd)
}
