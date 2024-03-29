package net.gearmaniacs.teamcode.utils

object PathAction {
    const val NO_ACTION = 0
    const val SLEEP_500 = 1 shl 0
    const val START_INTAKE = 1 shl 1
    const val STOP_INTAKE = 1 shl 2
    const val ATTACH_FOUNDATION = 1 shl 3
    const val PREPARE_ATTACH_FOUNDATION = 1 shl 4
    const val DETACH_FOUNDATION = 1 shl 5
    const val ATTACH_GRIPPER = 1 shl 6
    const val RELEASE_GRIPPER = 1 shl 7
    const val EXTEND_OUTTAKE = 1 shl 8
    const val RETRACT_OUTTAKE = 1 shl 9
}

data class PathPoint(
    val x: Double,
    val y: Double,
    val angle: Double,
    val moveError: Double = 2.0,
    val turnError: Double = Math.toRadians(5.0),
    val action: Int = PathAction.NO_ACTION
) {
    override fun toString(): String = buildString {
        append("PathPoint(")
        append(String.format("%.2f", x))
        append(String.format("%.2f", y))
        append(String.format("%.3f", angle))
        append(')')
    }
}
