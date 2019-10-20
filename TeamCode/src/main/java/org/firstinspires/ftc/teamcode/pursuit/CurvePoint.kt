package org.firstinspires.ftc.teamcode.pursuit

data class CurvePoint(
    var x: Double,
    var y: Double,
    val moveSpeed: Double,
    val turnSpeed: Double,
    val followSpeed: Double,
    val followDistance: Double
    // The rest could be added in the future to improve the algorithm
    // val pointLength: Double = 0.0,
    // val slowDownTurnRadians: Double = 0.0,
    // val slowDownTurnAmount: Double = 0.0
) {

    fun toPoint() = Point(x, y)

    fun setPoint(point: Point) {
        x = point.x
        y = point.y
    }
}
