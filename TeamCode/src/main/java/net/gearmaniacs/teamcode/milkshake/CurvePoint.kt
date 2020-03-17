package net.gearmaniacs.teamcode.milkshake

data class CurvePoint(
    val x: Double,
    val y: Double,
    val followDistance: Double,
    val moveSpeed: Double = 1.0,
    val turnSpeed: Double = 1.0,
    val preferredAngle: Double = 0.0
    // The rest could be added in the future to improve the algorithm
    // val pointLength: Double,
    // val slowDownTurnRadians: Double,
    // val slowDownTurnAmount: Double
) {
    fun toPoint() = Point(x, y)
}

data class Point(
    var x: Double,
    var y: Double
)

data class RobotLocation(
    val x: Double,
    val y: Double,
    val angle: Double
)
