package net.gearmaniacs.teamcode.drive

import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints

object Drive {
    const val MAX_VEL = 145.0
    const val MAX_ACC = 200.0
    const val TICKS = 537.6
    const val DIAMETER = 10
    const val MAX_RPM = 340
    const val TRACK_WIDTH = 19.1
    const val MOTOR_VELOCITY_F = 32767 / (MAX_RPM * TICKS / 60.0)

    var BASE_CONSTRAINTS = DriveConstraints(
        MAX_VEL, MAX_ACC, 100.0,
        Math.toRadians(180.0), Math.toRadians(180.0), 0.0
    )

    const val kV = 1.0 / MAX_VEL
    const val kA = 1.0 / MAX_ACC
    const val kStatic = 0.0

    fun cmToTicks(cm: Double) = cm * TICKS / (DIAMETER * Math.PI)

    fun ticksToCm(ticks: Int) = (ticks * DIAMETER * Math.PI) / TICKS

    fun ticksToCm(ticks: Double) = ticksToCm(ticks.toInt())
}
