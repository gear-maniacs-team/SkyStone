package org.firstinspires.ftc.teamcode.pid

import org.firstinspires.ftc.robotcore.internal.system.Misc

abstract class AbstractPidController(
    var Kp: Double,
    var Ki: Double,
    var Kd: Double
) {
    var setPoint: Double = 0.0

    abstract fun compute(input: Double): Double

    override fun toString(): String = Misc.formatForUser("P=%f I=%f D=%f", Kp, Ki, Kd)
}
