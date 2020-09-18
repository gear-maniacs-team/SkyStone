package net.gearmaniacs.teamcode.utils

import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.milkshake.Point
import java.lang.RuntimeException
import kotlin.math.*

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

        val slope = (endPoint.y - startPoint.y) / (endPoint.x - startPoint.x)
        val slopeSq = slope * slope

        // Subtract the offset of the circle
        val x = startPoint.x - circleCenter.x
        val y = startPoint.y - circleCenter.y

        val quadraticA = 1.0 + slopeSq
        val quadraticB = (2.0 * slope * y) - (2.0 * slopeSq * x)
        val quadraticC = (slopeSq * (x * x)) - (2.0 * y * slope * x) + (y * y) - (radius * radius)

        val allPoints = ArrayList<Point>(2)

        try {
            val delta = sqrt((quadraticB * quadraticB) - (4.0 * quadraticA * quadraticC))
            if (delta.isNaN())
                throw RuntimeException("Delta is NaN. Intersections couldn't be computed")

            var xRoot1 = (-quadraticB + delta) / (2.0 * quadraticA)
            var yRoot1 = slope * (xRoot1 - x) + y

            // Add back the offset of the circle
            xRoot1 += circleCenter.x
            yRoot1 += circleCenter.y

            // TODO: Should we instead compute the min/max of y?
            val minX = min(startPoint.x, endPoint.x)
            val maxX = max(startPoint.x, endPoint.x)

            if (minX < xRoot1 && xRoot1 < maxX)
                allPoints.add(Point(xRoot1, yRoot1))

            var xRoot2 = (-quadraticB - delta) / (2.0 * quadraticA)
            var yRoot2 = slope * (xRoot2 - x) + y

            // Add back the offset of the circle
            xRoot2 += circleCenter.x
            yRoot2 += circleCenter.y

            if (minX < xRoot2 && xRoot2 < maxX)
                allPoints.add(Point(xRoot2, yRoot2))
        } catch (e: Exception) {
            e.printStackTrace()
        }

        return allPoints
    }

    fun expo(input: Double, expoFactor: Double): Double =
        expoFactor * input * input * input + (1.0 - expoFactor) * input

    fun rotateVector(x: Double, y: Double, radians: Radians): Pair<Double, Double> {
        val sinAngle = sin(radians.value)
        val cosAngle = cos(radians.value)
        return (x * cosAngle - y * sinAngle) to (x * sinAngle + y * cosAngle)
    }
}
