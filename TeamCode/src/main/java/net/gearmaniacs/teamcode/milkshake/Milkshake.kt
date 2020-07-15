package net.gearmaniacs.teamcode.milkshake

import android.util.Log
import com.qualcomm.robotcore.util.Range
import net.gearmaniacs.teamcode.utils.MathUtils
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.min
import kotlin.math.sin
import kotlin.math.sqrt

class Milkshake(path: List<CurvePoint>) {

    private val pathPoints = path.toMutableList()

    fun followCurve(robotLocation: RobotLocation): Result {
        val destination = getFollowPointPath(robotLocation)

        val destinationPoint = destination.toPoint()
        Log.v("Follow Curve Dest", destinationPoint.toString())

        return goToPosition(
            robotLocation,
            destinationPoint,
            destination.moveSpeed,
            destination.turnSpeed,
            destination.preferredAngle
        )
    }

    private fun getFollowPointPath(robotLocation: RobotLocation): CurvePoint {
        val currentLocation = Point(robotLocation.x, robotLocation.y)

        var foundIntersections = false
        var pathOffset = 0
        var maxPointsToSkip = 3
        var result = pathPoints.first()

        for (i in 1..min(maxPointsToSkip, pathPoints.size - 1)) {
            val lineStart = pathPoints[i - 1]
            val lineEnd = pathPoints[i]

            val intersections: List<Point> = MathUtils.linesCircleIntersections(
                currentLocation,
                lineEnd.followDistance,
                lineStart.toPoint(),
                lineEnd.toPoint()
            )

            // Remove previous points
            if (intersections.isEmpty()) {
                if (!foundIntersections) {
                    ++pathOffset
                    continue
                }
            } else
                foundIntersections = true

            if (i == pathPoints.size - 1) {
                // lineEnd is the last point of the path
                val (x, y) = currentLocation
                val (endX, endY) = lineEnd

                // if we are closer than lookahead distance to the end, set it as the lookahead
                if (sqrt((endX - x) * (endX - x) + (endY - y) * (endY - y)) <= lineEnd.followDistance)
                    return lineEnd
            }

            if (i == maxPointsToSkip && !foundIntersections)
                ++maxPointsToSkip

            var closestAngle = Double.MAX_VALUE

            intersections.forEach {
                val absoluteAngle = atan2(it.y - robotLocation.y, it.x - robotLocation.x)
                val deltaAngle = abs(MathUtils.angleWrap(absoluteAngle - robotLocation.angle))

                if (deltaAngle < closestAngle) {
                    closestAngle = deltaAngle
                    result = lineEnd.copy(x = it.x, y = it.y)
                }
            }
        }

        for (i in 0 until pathOffset)
            pathPoints.removeAt(pathOffset)

        return result
    }

    data class Result(val xPower: Double, val yPower: Double, val rotationPower: Double)

    companion object {
        fun goToPosition(
            robotLocation: RobotLocation,
            destination: Point,
            movementSpeed: Double,
            turnSpeed: Double,
            preferredAngle: Double
        ): Result {
            val x = destination.x - robotLocation.x
            val y = destination.y - robotLocation.y

            val distance = hypot(x, y)
            val absoluteAngleToPoint = atan2(y, x)
            val relativeAngleToPoint = MathUtils.angleWrap(absoluteAngleToPoint - (robotLocation.angle - Math.PI / 2))

            val relativeDistanceX = sin(relativeAngleToPoint) * distance
            val relativeDistanceY = cos(relativeAngleToPoint) * distance
            val totalRelativeDistance = abs(relativeDistanceX) + abs(relativeDistanceY)

            // Normalize the power on x/y axes
            val movementXPower = relativeDistanceX / totalRelativeDistance
            val movementYPower = relativeDistanceY / totalRelativeDistance

            val targetX = movementXPower * movementSpeed
            val targetY = movementYPower * movementSpeed

            // Do not turn the robot if it is too close to the target
            val targetRotation = if (distance < 10)
                0.0
            else {
                val relativeTurnAngle = relativeAngleToPoint - Math.PI + preferredAngle
                Range.clip(relativeTurnAngle / (Math.PI / 6), -1.0, 1.0) * turnSpeed
            }

            return Result(targetX, targetY, targetRotation)
        }
    }
}
