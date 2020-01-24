package net.gearmaniacs.teamcode.utils

data class PathPoint(
    val cmX: Double,
    val cmY: Double,
    val angle: Double,
    val moveSpeed: Double = 0.0,
    val turnSpeed: Double = 0.0,
    val action: Int = ACTION_NONE
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

            if (action != ACTION_NONE) {
                append(", action = PathPoint.")
                append(getActionName(action))
            }

            append(')')
        }
    }

    companion object {
        const val ACTION_NONE = 0
        const val ACTION_START_INTAKE = 1
        const val ACTION_STOP_INTAKE = 2
        const val ACTION_ATTACH_FOUNDATION = 3
        const val ACTION_DETACH_FOUNDATION = 4
        const val ACTION_SIMPLE_OUTTAKE = 5

        fun getActionName(action: Int) = when (action) {
            ACTION_NONE -> ::ACTION_NONE.name
            ACTION_START_INTAKE -> ::ACTION_START_INTAKE.name
            ACTION_STOP_INTAKE -> ::ACTION_STOP_INTAKE.name
            ACTION_ATTACH_FOUNDATION -> ::ACTION_ATTACH_FOUNDATION.name
            ACTION_DETACH_FOUNDATION -> ::ACTION_DETACH_FOUNDATION.name
            ACTION_SIMPLE_OUTTAKE -> ::ACTION_SIMPLE_OUTTAKE.name
            else -> IllegalArgumentException("Unsupported Action")
        }
    }
}
