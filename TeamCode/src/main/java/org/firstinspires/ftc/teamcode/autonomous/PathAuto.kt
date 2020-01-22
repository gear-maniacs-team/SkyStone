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
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.sin

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
        PathPoint(cmX = -106.91124687759908, cmY = 155.649507904549, angle = 1.9749674953960017),
        PathPoint(
            cmX = -14.00431359903669,
            cmY = 119.8044292930338,
            angle = 1.9348230166720977,
            action = PathPoint.ACTION_START_INTAKE
        ),
        PathPoint(cmX = -102.77821898198754, cmY = 166.25925114813768, angle = 4.4766023800856285),
        PathPoint(
            cmX = 380.2288877851951,
            cmY = 158.22497199722312,
            angle = 4.844452577234671,
            action = PathPoint.ACTION_STOP_INTAKE
        ),
        PathPoint(cmX = 383.4900979454124, cmY = 154.532476900059, angle = 9.320350668219872),
        PathPoint(cmX = 381.13815906084494, cmY = 191.8652019842255, angle = 9.330069857805647),
        PathPoint(
            cmX = 432.3852251854324,
            cmY = 207.26439619152274,
            angle = 4.304614981763193,
            action = PathPoint.ACTION_ATTACH_FOUNDATION
        ),
        PathPoint(
            cmX = 490.63666967696616,
            cmY = 252.84953765221667,
            angle = 4.371592875213077,
            action = PathPoint.ACTION_DETACH_FOUNDATION
        )
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

            strafe()
            move()
            rotate()
            performAction(it.action)

            pointIndex++
        }

        intake.left.power = 0.0
        intake.right.power = -0.0

        robot.stop()
    }

    private fun goToPoint() {
        xPid.setPoint = RobotPos.targetX
        yPid.setPoint = RobotPos.targetY

        xPid.reset()
        yPid.reset()

        var count = 0
        val startTime = System.currentTimeMillis()

        while (opModeIsActive()) {
            val x = xPid.compute(RobotPos.currentX)
            val y = yPid.compute(RobotPos.currentY)

            with(telemetry) {
                addData("Current X", RobotPos.currentX)
                addData("Current Y", RobotPos.currentY)
                addData("Current , angle", RobotPos.currentAngle)
                addData("--", "--")
                addData("Target X", RobotPos.targetX)
                addData("Target Y", RobotPos.targetY)
                addData("Target , angle", RobotPos.targetAngle)
                addData("--", "--")
                addData("PID X", x)
                addData("PID Y", y)
                update()
            }

            if (!movement(x, y) && count > 5)
                break
            if (x smaller 0.2 && y smaller 0.2 && count > 5)
                break

            if (System.currentTimeMillis() - startTime > 3000)
                break

            Thread.sleep(5)
            count++
        }
    }

    private fun strafe() {
        var count = 0
        xPid.setPoint = RobotPos.targetX

        xPid.reset()

        val startTime = System.currentTimeMillis()

        while (opModeIsActive()) {
            val x = xPid.compute(RobotPos.currentX)

            with(telemetry) {
                addData("Current X", RobotPos.currentX)
                addData("Current Y", RobotPos.currentY)
                addData("Current , angle", RobotPos.currentAngle)
                addData("--", "--")
                addData("Target X", RobotPos.targetX)
                addData("Target Y", RobotPos.targetY)
                addData("Target , angle", RobotPos.targetAngle)
                addData("--", "--")
                addData("PID X", x)
                update()
            }

            movement(x, 0.0)

            if (x smaller 0.1 && count > 5)
                break

            if (System.currentTimeMillis() - startTime > 3000)
                break

            Thread.sleep(5)
        }
    }

    private fun move() {
        var count = 0
        yPid.setPoint = RobotPos.targetY

        yPid.reset()

        val startTime = System.currentTimeMillis()

        while (opModeIsActive()) {
            val y = yPid.compute(RobotPos.currentY)

            with(telemetry) {
                addData("Current X", RobotPos.currentX)
                addData("Current Y", RobotPos.currentY)
                addData("Current , angle", RobotPos.currentAngle)
                addData("--", "--")
                addData("Target X", RobotPos.targetX)
                addData("Target Y", RobotPos.targetY)
                addData("Target , angle", RobotPos.targetAngle)
                addData("--", "--")
                addData("PID Y", y)
                update()
            }

            movement(0.0, y)

            if (y smaller 0.1 && count > 5)
                break

            if (System.currentTimeMillis() - startTime > 3000)
                break

            count++
            Thread.sleep(5)
        }
    }

    private fun rotate() {
        if (RobotPos.currentAngle == RobotPos.targetAngle) return

        var count = 0

        rotationPid.setPoint = RobotPos.targetAngle
        rotationPid.reset()

        while (opModeIsActive()) {
            val rotation = rotationPid.compute(RobotPos.currentAngle)

            with(wheels) {
                rightFront.power = rotation
                leftFront.power = rotation
                rightBack.power = rotation
                leftBack.power = rotation
            }

            if (rotation smaller 0.1 && count > 5)
                break

            count++
            Thread.yield()
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
