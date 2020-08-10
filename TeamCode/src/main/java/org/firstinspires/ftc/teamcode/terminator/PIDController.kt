package org.firstinspires.ftc.teamcode.terminator

import net.gearmaniacs.teamcode.utils.RobotClock
import net.gearmaniacs.teamcode.utils.extensions.epsilonEquals
import kotlin.math.abs
import kotlin.math.sign

class PIDController(
    var kp: Double,
    var ki: Double,
    var kd: Double
) {
    var setPoint = 0.0
    private var input = 0.0
    private var result = 0.0
    private var error = 0.0
    private var lastError = 0.0
    private var currentTime = 0L
    private var previousTime = 0L
    private var totalError = 0.0
    var firstRun = true

    var minInput = 0.0
        private set
    var maxInput = 0.0
        private set
    var minOutput = 0.0
        private set
    var maxOutput = 0.0
        private set

    private fun calculate() {
        if (firstRun) {
            previousTime = RobotClock.millis()
            firstRun = false
            lastError = error
            result = 0.0
            return
        }

        error = setPoint - input
        currentTime = RobotClock.millis()


        val deltaTime = currentTime - previousTime
        if (deltaTime == 0L){
            result = 0.0
            return
        }
        val derivative = (error - lastError) / deltaTime

        if ((abs(totalError + error) * ki < maxOutput &&
                    abs(totalError + error) * ki > minOutput) || sign(totalError) != sign(error)
        ) {
            totalError += 0.5 * (error + lastError) * deltaTime
        }

        result = kp * error + kd * derivative + ki * totalError
        result = if (result epsilonEquals 0.0) 0.0 else result
        lastError = error
        previousTime = currentTime
    }

    fun performPID(_input: Double): Double {
        input = _input
        calculate()
        return result
    }

    fun setInputRange(min: Double, max: Double) {
        require(min < max)

        minInput = min
        maxInput = max
    }

    fun setOutputRange(min: Double, max: Double) {
        require(min < max)

        minOutput = min
        maxOutput = max
    }
}
