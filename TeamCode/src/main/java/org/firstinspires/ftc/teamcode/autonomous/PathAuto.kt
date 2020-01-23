package org.firstinspires.ftc.teamcode.autonomous

import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.RobotPos
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.motors.Intake
import org.firstinspires.ftc.teamcode.motors.Wheels
import org.firstinspires.ftc.teamcode.pid.PidController
import org.firstinspires.ftc.teamcode.sensors.Encoder
import org.firstinspires.ftc.teamcode.utils.*
import kotlin.math.*

@Autonomous(name = "Path")
open class PathAuto : LinearOpMode() {

    private val robot = TeamRobot()
    private val wheels = Wheels()
    private val encoder = Encoder()
    private val intake = Intake()

    private val xPid = PidController(0.45, 0.0, 20.0)
    private val yPid = PidController(0.45, 0.0, 20.0)
    private val rotationPid = PidController(2.0, 0.0, 0.05)

    private lateinit var foundationLeft: Servo
    private lateinit var foundationRight: Servo

    private var pointIndex = 0

    //private val path by fastLazy { PathParser.parseAsset(hardwareMap.appContext, "path/test.json") }
    private val path = listOf(
        PathPoint(cmX = -40.93695166138619, cmY = 51.45290146248186, angle = 0.6367750764304464),
        PathPoint(cmX = -16.314492652020558, cmY = 85.76970429920864, angle = 0.5895413735468702),
        PathPoint(cmX = -44.609871622265395, cmY = 62.622368725130976, angle = 1.4405223484493297),
        PathPoint(cmX = -177.9415202761401, cmY = 43.027514277998186, angle = 1.5176730762591641),
        PathPoint(cmX = -188.60256298054585, cmY = 45.10308159168387, angle = 3.03809147951157),
        PathPoint(cmX = -189.4154104532654, cmY = 59.47519721960557, angle = 3.0633625664492796),
        PathPoint(cmX = -158.4471993648067, cmY = 58.55019882874637, angle = 3.0078224895861823),
        PathPoint(cmX = -161.76232529560542, cmY = 59.12988456929493, angle = 1.5591345531569811),
        PathPoint(cmX = -185.84933846927575, cmY = 59.48316474960153, angle = 1.488952732329971),
        PathPoint(cmX = -137.20332873835304, cmY = 53.99830296887041, angle = 1.587291753087581),
        PathPoint(cmX = -85.54350723332526, cmY = 56.35628266990834, angle = 1.5456894901901252)
    )

    override fun runOpMode() {
        robot.init(hardwareMap, listOf(wheels, intake, encoder), listOf(encoder))
        RobotPos.resetAll()

        xPid.setOutputRange(-MOTOR_SPEED, MOTOR_SPEED)
        yPid.setOutputRange(-MOTOR_SPEED, MOTOR_SPEED)
        rotationPid.setOutputRange(-MOTOR_SPEED, MOTOR_SPEED)

        foundationLeft = hardwareMap.getDevice("foundation_left")
        foundationRight = hardwareMap.getDevice("foundation_right")

        while (!isStarted) {
            telemetry.addData("Status", "Waiting for start")
            telemetry.addData("Path Size", path.size)
            telemetry.update()
        }
        robot.start()

        actionIntake(true)

        path.forEach {
            RobotPos.targetX = it.cmX
            RobotPos.targetY = it.cmY
            RobotPos.targetAngle

            goToPoint()
            performAction(it.action)

            pointIndex++
        }

        intake.left.power = 0.0
        intake.right.power = -0.0

        robot.stop()
    }

    private fun goToPoint() {
        var distanceToTargetX = RobotPos.targetX - RobotPos.currentX
        var distanceToTargetY = RobotPos.targetY - RobotPos.currentY

        val distance = Math.hypot(distanceToTargetX, distanceToTargetY)

        while (opModeIsActive() && distance epsilonEquals 0.0) {
            distanceToTargetX = RobotPos.targetX - RobotPos.currentX
            distanceToTargetY = RobotPos.targetY - RobotPos.currentY

            val robotMovementAngle = Math.atan2(distanceToTargetX, distanceToTargetY)
            val motionX = sin(robotMovementAngle) * MOTOR_SPEED
            val motionY = cos(robotMovementAngle) * MOTOR_SPEED
            val pivotCorrection = RobotPos.targetAngle - RobotPos.currentAngle
            //movement()

            with(telemetry) {
                addData("Current X", RobotPos.currentX)
                addData("Current Y", RobotPos.currentY)
                addData("Current , angle", RobotPos.currentAngle)
                addData("--", "--")
                addData("Target X", RobotPos.targetX)
                addData("Target Y", RobotPos.targetY)
                addData("Target , angle", RobotPos.targetAngle)
                update()
            }

            Thread.sleep(5)
        }
    }

    private fun movement(x: Double, y: Double): Boolean {
        val magnitude = hypot(x, y) * MOTOR_SPEED

        if (magnitude smaller 0.05) return false

        val angle = atan2(y, x) - Math.PI / 2

        val speedX = magnitude * sin(angle + Math.PI / 4)
        val speedY = magnitude * sin(angle - Math.PI / 4)

        with(wheels) {
            rightFront.power = -speedX
            leftFront.power = -speedY
            rightBack.power = speedY
            leftBack.power = speedX
        }

        return true
    }

    private fun performAction(action: Int) {
        when (action) {
            PathPoint.ACTION_NONE -> return
            PathPoint.ACTION_START_INTAKE -> actionIntake(true)
            PathPoint.ACTION_STOP_INTAKE -> actionIntake(false)
            PathPoint.ACTION_ATTACH_FOUNDATION -> foundation(true)
            PathPoint.ACTION_DETACH_FOUNDATION -> foundation(false)
            else -> Log.e(TAG, "Unsupported Action!")
        }
    }

    private fun actionIntake(start: Boolean) {
        val power = if (start) 1.0 else 0.0
        intake.left.power = power
        intake.right.power = -power
    }

    private fun foundation(attach: Boolean) {
        foundationLeft.position = if (attach) 0.0 else 1.0
        foundationRight.position = if (attach) 1.0 else 0.0
    }

    private companion object {
        const val TAG = "PathAuto"
        const val MOTOR_SPEED = 0.7

        infix fun Double.smaller(other: Double) = abs(this) < other
    }
}
