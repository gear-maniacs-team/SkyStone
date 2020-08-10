package org.firstinspires.ftc.teamcode.terminator

import android.util.Log
import net.gearmaniacs.teamcode.RobotPos
import kotlin.math.cos
import kotlin.math.sin

class PIDSystem {

    private val XPID = PIDController(0.03, 0.0, 3.2)
    private val YPID = PIDController(0.03, 0.0, 3.2)
    private val APID = PIDController(1.0, 0.0, 4.0)

    var pathPoints = mutableListOf<PathPoint>()
    private val movementApprox = 5.0
    private val angleApprox = 3.0
    var currentPoint = 0
        private set
    private var isWaiting = false
    var isActive = true

    fun init() {
        setPoint(pathPoints[currentPoint])
    }

    fun setOutputRanges(min: Double, max: Double) {
        XPID.setOutputRange(min, max)
        YPID.setOutputRange(min, max)
        APID.setOutputRange(min, max)
    }

    fun pathSize() = pathPoints.size

    fun run(wheels: Wheels) {
        Log.v("PIDSystem", "currentPoint = $currentPoint")
        Log.v("PIDSystem", "isWaiting = $isWaiting")

        if (hasReachedPoint(pathPoints[currentPoint])) {

            if (currentPoint < pathPoints.size) {
                currentPoint++
                if (currentPoint == pathPoints.size) {
                    return
                }
                setPoint(pathPoints[currentPoint])
            }

        }

        if (isActive) {
            val currentAngle = RobotPos.currentAngle
            val xResult = XPID.performPID(RobotPos.currentX)
            val yResult = YPID.performPID(RobotPos.currentY)
            val aResult = APID.performPID(currentAngle)

            val sinOrientation = sin(currentAngle)
            val cosOrientation = cos(currentAngle)
            val fieldOrientedX = xResult * cosOrientation - yResult * sinOrientation
            val fieldOrientedY = xResult * sinOrientation + yResult * cosOrientation

            wheels.frontLeft.power = fieldOrientedY + aResult + fieldOrientedX
            wheels.frontRight.power = fieldOrientedY - aResult - fieldOrientedX
            wheels.backLeft.power = fieldOrientedY + aResult - fieldOrientedX
            wheels.backRight.power = fieldOrientedY - aResult + fieldOrientedX
        }
    }

    fun setPoint(point: PathPoint) {
        XPID.setPoint = point.X
        YPID.setPoint = point.Y
        APID.setPoint = point.angle
    }

    fun addPoint(point: PathPoint) {
        pathPoints.add(point)
    }

    fun addPoints(vararg points: PathPoint) {
        for (point in points) {
            addPoint(point)
        }
    }

    private fun isApprox(val1: Double, val2: Double, approximation: Double): Boolean {
        return (val1 > val2 - approximation && val1 < val2 + approximation)
    }

    fun hasReachedPoint(point: PathPoint): Boolean {
        return isApprox(point.X, RobotPos.currentX, movementApprox)
                && isApprox(point.Y, RobotPos.currentY, movementApprox)
                && isApprox(point.angle, RobotPos.currentAngle, Math.toRadians(angleApprox))
    }
}
