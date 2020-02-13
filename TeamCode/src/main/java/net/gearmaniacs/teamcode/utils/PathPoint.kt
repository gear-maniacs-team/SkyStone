package net.gearmaniacs.teamcode.utils

data class PathPoint(
    val cmX: Double,
    val cmY: Double,
    val angle: Double,
    val moveSpeed: Double = 1.0,
    val turnSpeed: Double = 1.0,
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
            append(')')
        }
    }

    companion object {
        const val ACTION_NONE = 0
        const val ACTION_START_INTAKE = 1 shl 0
        const val ACTION_STOP_INTAKE = 1 shl 1
        const val ACTION_ATTACH_FOUNDATION = 1 shl 2
        const val ACTION_DETACH_FOUNDATION = 1 shl 3
        const val ACTION_SIMPLE_OUTTAKE = 1 shl 4

        fun getActionName(action: Int) = when (action) {
            ACTION_NONE -> ::ACTION_NONE.name
            ACTION_START_INTAKE -> ::ACTION_START_INTAKE.name
            ACTION_STOP_INTAKE -> ::ACTION_STOP_INTAKE.name
            ACTION_ATTACH_FOUNDATION -> ::ACTION_ATTACH_FOUNDATION.name
            ACTION_DETACH_FOUNDATION -> ::ACTION_DETACH_FOUNDATION.name
            ACTION_SIMPLE_OUTTAKE -> ::ACTION_SIMPLE_OUTTAKE.name
            else -> "Unsupported Path Action"
        }
    }
}
