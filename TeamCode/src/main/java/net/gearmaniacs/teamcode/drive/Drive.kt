package net.gearmaniacs.teamcode.drive

import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints

object Drive {
    const val MAX_VEL = 150.0 / 2
    const val MAX_ACC = 200.0
    const val MAX_JERK = 150.0
    const val MAX_VEL_ANG = 3.0 * Math.PI / 2
    const val MAX_ACC_ANG = 2 * Math.PI
    const val MAX_JERK_ANG = 4 * Math.PI

    const val TICKS = 537.6
    const val DIAMETER = 10
    const val MAX_RPM = 340
    const val TRACK_WIDTH = 19.1
    const val MOTOR_VELOCITY_F = 32767 / (MAX_RPM * TICKS / 60.0)

    val BASE_CONSTRAINTS = DriveConstraints(
        MAX_VEL, MAX_ACC, MAX_JERK,
        MAX_VEL_ANG, MAX_ACC_ANG, MAX_JERK_ANG
    )

    const val kV = 1.0 // MAX_VEL
    const val kA = 0.0
    const val kStatic = 0.0

    fun cmToTicks(cm: Double) = cm * TICKS / (DIAMETER * Math.PI)

    fun ticksToCm(ticks: Int) = (ticks * DIAMETER * Math.PI) / TICKS

    fun ticksToCm(ticks: Double) = ticksToCm(ticks.toInt())
}
