package org.firstinspires.ftc.teamcode.pursuit

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

fun angleWrap(angle: Double): Double {
    var newAngle = angle

    while (newAngle < -Math.PI)
        newAngle += 2 * Math.PI

    while (angle > Math.PI)
        newAngle -= 2 * Math.PI

    return newAngle
}

fun linesCircleIntersections(
    circleCenter: Point, radius: Double,
    linePoint1: Point, linePoint2: Point
): List<Point> {
    if (abs(linePoint1.y - linePoint2.y) < 0.003)
        linePoint1.y = linePoint2.y + 0.003
    if (abs(linePoint1.x - linePoint2.x) < 0.003)
        linePoint1.x = linePoint2.x + 0.003

    val m1 = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x)

    val quadraticA = 1.0 + m1 * m1

    val x1 = linePoint1.x - circleCenter.x
    val y1 = linePoint1.y - circleCenter.y

    val quadraticB = (2.0 * m1 * y1) - (2.0 * (m1 * m1) * x1)
    val quadraticC = ((m1 * m1) * (x1 * x1)) - (2.0 * y1 * m1 * x1) + (y1 * y1) - (radius * radius)

    val allPoints = ArrayList<Point>(2)

    try {
        var xRoot1 =
            (-quadraticB + sqrt((quadraticB * quadraticB) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA)
        var yRoot1 = m1 * (xRoot1 - x1) + y1

        // Add back the offset of the circle
        xRoot1 += circleCenter.x
        yRoot1 += circleCenter.y

        val minX = min(linePoint1.x, linePoint2.x)
        val maxX = max(linePoint1.x, linePoint2.x)

        if (xRoot1 > minX && xRoot1 < maxX)
            allPoints.add(Point(xRoot1, yRoot1))

        var xRoot2 =
            (-quadraticB - sqrt((quadraticB * quadraticB) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA)
        var yRoot2 = m1 * (xRoot2 - x1) + y1

        xRoot2 += circleCenter.x
        yRoot2 += circleCenter.y

        if (xRoot2 > minX && xRoot2 < maxX)
            allPoints.add(Point(xRoot2, yRoot2))
    } catch (e: Exception) {
    }

    return allPoints
}
