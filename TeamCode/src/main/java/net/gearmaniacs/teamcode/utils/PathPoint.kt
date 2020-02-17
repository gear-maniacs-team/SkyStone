package net.gearmaniacs.teamcode.utils

data class PathPoint(
    val cmX: Double,
    val cmY: Double,
    val angle: Double,
    val moveSpeed: Double = 1.0,
    val turnSpeed: Double = 1.0,
    val action: Int = PathAction.NO_ACTION
) {

    override fun toString(): String {
        return buildString {
            append("PathPoint(")
            append("cmX = ")
            append(cmX)
            append(", cmY = ")
            append(cmY)
            append(", angle = ")
            append(angle)
            append(')')
        }
    }
}

object PathAction {
    const val NO_ACTION = 0
    const val START_INTAKE = 1 shl 0
    const val STOP_INTAKE = 1 shl 1
    const val ATTACH_FOUNDATION = 1 shl 2
    const val DETACH_FOUNDATION = 1 shl 3
    const val EXTEND_OUTTAKE = 1 shl 4
    const val RETRACT_OUTTAKE = 1 shl 5
    const val ATTACH_GRIPPER = 1 shl 6
    const val RELEASE_GRIPPER = 1 shl 7

    fun getActionName(action: Int) = when (action) {
        NO_ACTION -> ::NO_ACTION.name
        START_INTAKE -> ::START_INTAKE.name
        STOP_INTAKE -> ::STOP_INTAKE.name
        ATTACH_FOUNDATION -> ::ATTACH_FOUNDATION.name
        DETACH_FOUNDATION -> ::DETACH_FOUNDATION.name
        EXTEND_OUTTAKE -> ::EXTEND_OUTTAKE.name
        RETRACT_OUTTAKE -> ::RETRACT_OUTTAKE.name
        ATTACH_GRIPPER -> ::ATTACH_GRIPPER.name
        RELEASE_GRIPPER -> ::RELEASE_GRIPPER.name
        else -> "Unsupported Path Action"
    }
}
