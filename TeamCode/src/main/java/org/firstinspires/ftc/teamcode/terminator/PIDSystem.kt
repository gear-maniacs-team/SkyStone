package org.firstinspires.ftc.teamcode.terminator
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamRobot
import net.gearmaniacs.teamcode.utils.RobotClock
import kotlin.math.sign

class PIDSystem {

    private val YPID = PIDController(0.04, 0.000005, 4.0)
    private val APID = PIDController(0.04, 0.000005, 4.0)//69420
    private val XPID = PIDController(0.05, 0.000005, 4.0)

    private var pathPoints = mutableListOf<PathPoint>()
    private val movementApprox = 1.0
    private val angleApprox = 2.5
    private var currentPoint = 0
    private val switchPointTime = 0.2
    private var isWaiting = false
    private var lastTime = 0.0

    fun init() {
        YPID.setOutputRange(-1.0, 1.0)
        APID.setOutputRange(-1.0, 1.0)
        XPID.setOutputRange(-1.0, 1.0)

        setPoint(pathPoints[currentPoint])
    }

    fun run(robot : TeamRobot, wheels : Wheels){

        println("currentPoint = $currentPoint")
        println("isWaiting = $isWaiting")

        if (hasReachedPoint(pathPoints[currentPoint]))
        {
            isWaiting = true
            lastTime = RobotClock.seconds()
            if (RobotClock.seconds() - lastTime > switchPointTime)
                isWaiting = false

            if (currentPoint < pathPoints.size - 1)
            {
                setPoint(pathPoints[currentPoint])
                currentPoint++
            }
            else
                robot.stop()
        }

        if (!isWaiting)
        {

            val yResult = YPID.performPID(RobotPos.currentY)
            val aResult = APID.performPID(Math.toDegrees(RobotPos.currentAngle))
            val xResult = XPID.performPID(RobotPos.currentX)

            wheels.frontLeft.power = yResult + aResult + xResult
            wheels.frontRight.power = yResult - aResult - xResult
            wheels.backLeft.power = yResult + aResult - xResult
            wheels.backRight.power = yResult - aResult + xResult
        }

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
                && isApprox(point.angle, RobotPos.currentAngle, angleApprox)
    }

}

