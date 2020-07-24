package org.firstinspires.ftc.teamcode.terminator
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamRobot
import net.gearmaniacs.teamcode.utils.RobotClock
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin

class PIDSystem {

    private val YPID = PIDController(0.03, 0.0, 3.2)
    private val APID = PIDController(1.0, 0.0, 4.0)//69420
    private val XPID = PIDController(0.03, 0.0, 3.2)

    private var pathPoints = mutableListOf<PathPoint>()
    private val movementApprox = 5.0
    private val angleApprox = 3.0
    var currentPoint = 0
        private set
    private val switchPointTime = 0.2
    private var isWaiting = false
    private var lastTime = RobotClock.seconds()

    fun init() {
        YPID.setOutputRange(-0.33, 0.33)
        APID.setOutputRange(-0.33, 0.33)
        XPID.setOutputRange(-0.33, 0.33)

        setPoint(pathPoints[currentPoint])
    }

    fun pathSize() = pathPoints.size

    fun run(wheels : Wheels){

        println("currentPoint = $currentPoint")
        println("isWaiting = $isWaiting")

        if (hasReachedPoint(pathPoints[currentPoint])) {

            if (currentPoint < pathPoints.size)
            {
                currentPoint++
                if(currentPoint == pathPoints.size){
                    return
                }
                setPoint(pathPoints[currentPoint])
            }

        }

            val yResult = YPID.performPID(RobotPos.currentY)
            val aResult = APID.performPID(RobotPos.currentAngle)
            val xResult = XPID.performPID(RobotPos.currentX)

            val currentAngle = RobotPos.currentAngle
            val sinOrientation = sin(currentAngle)
            val cosOrientation = cos(currentAngle)

            val fieldOrientedX = xResult * cosOrientation - yResult * sinOrientation
            val fieldOrientedY = xResult * sinOrientation + yResult * cosOrientation

            wheels.frontLeft.power = fieldOrientedY + aResult + fieldOrientedX
            wheels.frontRight.power = fieldOrientedY - aResult - fieldOrientedX
            wheels.backLeft.power = fieldOrientedY + aResult - fieldOrientedX
            wheels.backRight.power = fieldOrientedY - aResult + fieldOrientedX

    }

    private fun setPoint(point: PathPoint) {
        YPID.setPoint = point.Y
        APID.setPoint = point.angle
        XPID.setPoint = point.X
    }

    fun addPoint(point: PathPoint){
        pathPoints.add(point)
    }

    fun addPoints(vararg points : PathPoint) {
        for(point in points){
            addPoint(point)
        }
    }

    private fun isApprox(val1 : Double, val2 : Double, approximation : Double) : Boolean{
        return (val1 > val2 - approximation && val1 < val2 + approximation)
    }

    private fun hasReachedPoint(point : PathPoint): Boolean {
        return isApprox(point.X, RobotPos.currentX, movementApprox)
                && isApprox(point.Y, RobotPos.currentY, movementApprox)
                && isApprox(point.angle, RobotPos.currentAngle, Math.toRadians(angleApprox))
    }

}

