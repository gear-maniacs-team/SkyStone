package org.firstinspires.ftc.teamcode.pursuit

import android.util.Log
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.RobotPos
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

object RobotMovement {

    // Store these here as they are needed in high-performance situations
    private val RADIANS_30 = Math.toRadians(30.0)
    private val RADIANS_90 = Math.toRadians(90.0)
    private val RADIANS_180 = Math.toRadians(180.0)

    fun followCurve(allPoints: List<CurvePoint>, preferredAngle: Double) {
        for (i in 0 until allPoints.size - 1) {
            val current = allPoints[i]
            val next = allPoints[i + 1]

            Log.v("Follow Curve", Point(current.x, current.y).toString() + " ; " + Point(next.x, next.y).toString())
        }

        val destination = getFollowPointPath(allPoints, Point(RobotPos.currentX, RobotPos.currentY), allPoints.first().followDistance)

        Log.v("Follow Curve Dest", Point(destination.x, destination.y).toString())

        goToPosition(destination.x, destination.y, destination.moveSpeed, preferredAngle, destination.turnSpeed)
    }

    private fun getFollowPointPath(pathPoints: List<CurvePoint>, robotLocation: Point, followRadius: Double): CurvePoint {
        val result = pathPoints.first().copy()

        for (i in 0 until pathPoints.size - 1) {
            val startLine = pathPoints[i]
            val endLine = pathPoints[i + 1]

            val intersections: List<Point> =
                linesCircleIntersections(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint())

            var closestAngle = 10_000_000.0

            intersections.forEach {
                val angle = atan2(it.y - RobotPos.currentY, it.x - RobotPos.currentX)
                val deltaAngle = abs(angleWrap(angle - RobotPos.currentAngle))

                if (deltaAngle < closestAngle) {
                    closestAngle = deltaAngle
                    result.setPoint(it)
                }
            }
        }

        return result
    }

    fun goToPosition(x: Double, y: Double, movementSpeed: Double, preferredAngle: Double, turnSpeed: Double) {
        val distanceToTarget = hypot(x - RobotPos.currentX, y - RobotPos.currentY)
        val absoluteAngleToTarget = atan2(y - RobotPos.currentY, x - RobotPos.currentX)
        val relativeAngleToPoint = angleWrap(absoluteAngleToTarget - (RobotPos.currentAngle - RADIANS_90))

        val relativeXToPoint = cos(relativeAngleToPoint) * distanceToTarget
        val relativeYToPoint = sin(relativeAngleToPoint) * distanceToTarget

        // Have normalized power
        val movementXPower = relativeXToPoint / (abs(relativeXToPoint + abs(relativeYToPoint)))
        val movementYPower = relativeYToPoint / (abs(relativeXToPoint + abs(relativeYToPoint)))

        RobotPos.targetX = movementXPower * movementSpeed
        RobotPos.targetY = movementYPower * movementSpeed

        RobotPos.targetAngle = if (distanceToTarget < 10)
            0.0
        else {
            val relativeTurnAngle = relativeAngleToPoint - RADIANS_180 + preferredAngle
            Range.clip(relativeTurnAngle / RADIANS_30, -1.0, 1.0) * turnSpeed
        }
    }
}