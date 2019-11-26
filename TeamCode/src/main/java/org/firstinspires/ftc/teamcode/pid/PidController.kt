package org.firstinspires.ftc.teamcode.pid

import org.firstinspires.ftc.robotcore.internal.system.Misc

abstract class PidController(
    var Kp: Double,
    var Ki: Double,
    var Kd: Double
) {
    var target: Double = 0.0

    abstract fun computePID(input: Double): Double

    override fun toString(): String = Misc.formatForUser("p=%f i=%f d=%f", Kp, Ki, Kd)
}
