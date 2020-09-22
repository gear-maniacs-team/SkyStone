package net.gearmaniacs.teamcode.drive

import net.gearmaniacs.teamcode.utils.Ranges
import net.gearmaniacs.teamcode.utils.RobotClock
import net.gearmaniacs.teamcode.utils.extensions.epsilonEquals
import net.gearmaniacs.teamcode.utils.extensions.millisToSeconds
import kotlin.math.abs
import kotlin.math.sign

/**
 * Here we go again...
 *
 * So our Team's Leader has expressed his dream last season to be able to use jerk limiting and therefore
 * MotionProfiling during TeleOp.
 * As far as I've looked nobody ever attempted this (or at least didn't share his/her experience) and when something like
 * this happens you know deep inside you that something is wrong.
 * (Well I didn't find anything except for maybe this: https://www.youtube.com/watch?v=oHg5SJYRHA0)
 *
 * Jokes aside, let's get down to business.
 *
 * I will not yet attempt to program a realtime S-curve profile generator because it has 7 stages and a lot
 * more physics. It's certainly possible, however it seems impractical since S-curves are mostly used by hospital
 * elevators and in other places where vibrations may be an issue and NOT for 20 kg robots designed to shoot foam disks
 * at a blue or red piece of plastic tubing. Trapezoidal profiles make the robot more aggressive than an s-curve but are
 * still much better than drivers doing whatever they want. After all such a profile spends most of the time with
 * constant acceleration / velocity, which means 0 m/s^3 jerk. (As long as the driver's Parkinson disease isn't too bad)
 *
 *
 *
 * Nevertheless, here is my thought process for generating a realtime trapezoidal profile:
 *
 * We know:
 *      The target acceleration
 *      The target deceleration
 *      How much time is needed to accelerate or decelerate to a certain velocity (because physix and meth):
 *          deltaT = (v-v0)/a
 *      Max Speed/Power
 *      Current Speed/Power
 *      Last Speed/Power
 *
 * We look at two consecutive reads of the input parameter (power or speed) and calculate the difference
 * This way we determine if the driver wants to accelerate, decelerate or keep the speed constant
 *
 * Scenario 1: The driver accelerates
 *      We start a timer at 0s (only at the transition between the last state and acceleration) and measure elapsed
 *      time.
 *      We compute next speed as v = currentVel + acc * elapsedTime.
 * Scenario 2: The driver is at constant velocity:
 *      We leave him live his life in peace and harmony because he is a great driver (basically never happens, except
 *      when robot is stationary)
 * Scenario 3: The driver decelerates
 *      analogous to acceleration.
 *
 *  A message for future generations of GM Programmers. If you're reading this then you're probably in some problematic
 *  situation. I recommend that you revise some physics before attempting to understand this. However, if you ever need
 *  any help you can call Gear Maniacs Tech Support at 0785 233 025 or 071 285 234. Or even better get in touch with our
 *  boss at 0729 758 610.
 *
 *  Have fun programming!
 */
class RealTimeMotionProfiler(
    private val targetAcceleration: Float,
    private val targetDeceleration: Float = -targetAcceleration,
    private val maxSpeed: Float = 1.0f
) {
    private var lastDirection = 0.0f
    private var lastTimeStamp = 0L
    private var lastCorrectedSpeed = 0.0f
    private var initialSpeed = 0.0f

    fun computeCorrectedPower(speed: Float): Float {
        var direction = sign(speed - lastCorrectedSpeed)

        if (abs(lastCorrectedSpeed - speed) < 0.09) {
            direction = 0.0f
            lastCorrectedSpeed = speed
        }

        val isTransitionPhase = !(lastDirection epsilonEquals direction)

        lastDirection = direction
        if (isTransitionPhase) {
            lastTimeStamp = RobotClock.millis()
            initialSpeed = lastCorrectedSpeed
            return lastCorrectedSpeed
        }

        val deltaTime =
            (RobotClock.millis() - lastTimeStamp).millisToSeconds()


        lastCorrectedSpeed = Ranges.clamp(
            when (direction) {
                1.0f -> initialSpeed + targetAcceleration * deltaTime
                -1.0f -> initialSpeed + targetDeceleration * deltaTime
                else -> lastCorrectedSpeed
            }, -maxSpeed, maxSpeed
        )

        return lastCorrectedSpeed

    }
}

fun test() {
    val realTimeMotionProfiler = RealTimeMotionProfiler(0.5f)
    realTimeMotionProfiler.computeCorrectedPower(0.0f)
    var timestamp = System.currentTimeMillis()
    while (System.currentTimeMillis() - timestamp < 5000) {
        println(realTimeMotionProfiler.computeCorrectedPower(1.0f))
        Thread.sleep(100)
    }
    timestamp = System.currentTimeMillis()
    while (System.currentTimeMillis() - timestamp < 5000) {
        println(realTimeMotionProfiler.computeCorrectedPower(-1.0f))
        Thread.sleep(100)
    }
    timestamp = System.currentTimeMillis()
    while (System.currentTimeMillis() - timestamp < 5000) {
        println(realTimeMotionProfiler.computeCorrectedPower(0.5f))
        Thread.sleep(100)
    }

    timestamp = System.currentTimeMillis()
    while (System.currentTimeMillis() - timestamp < 5000) {
        println(realTimeMotionProfiler.computeCorrectedPower(0.0f))
        Thread.sleep(100)
    }
}
