package net.gearmaniacs.teamcode.utils

import net.gearmaniacs.teamcode.pursuit.Point
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

object MathUtils {

    const val EPSILON = 1e-6

    // wrapping in interval [-PI, PI]
    fun angleWrap(radians: Double): Double {
        var newAngle = (radians + Math.PI) % (2 * Math.PI)
        if (newAngle < 0)
            newAngle += 2 * Math.PI
        return newAngle - Math.PI
    }

    fun linesCircleIntersections(
        circleCenter: Point,
        radius: Double,
        startPoint: Point,
        endPoint: Point
    ): List<Point> {
        if (abs(startPoint.y - endPoint.y) < 0.003)
            startPoint.y = endPoint.y + 0.003
        if (abs(startPoint.x - endPoint.x) < 0.003)
            startPoint.x = endPoint.x + 0.003

        val m1 = (endPoint.y - startPoint.y) / (endPoint.x - startPoint.x)

        val quadraticA = 1.0 + m1 * m1

        val x1 = startPoint.x - circleCenter.x
        val y1 = startPoint.y - circleCenter.y

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

            val minX = min(startPoint.x, endPoint.x)
            val maxX = max(startPoint.x, endPoint.x)

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

    fun expo(input: Double, expoFactor: Double): Double =
        expoFactor * input * input * input + (1 - expoFactor) * input
}

infix fun Double.epsilonEquals(other: Double) = abs(this - other) < MathUtils.EPSILON

infix fun Double.smaller(other: Double) = abs(this) < other
