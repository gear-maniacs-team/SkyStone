package org.firstinspires.ftc.teamcode.autonomous

import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.robot.Robot
import org.firstinspires.ftc.teamcode.RobotPos
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.motors.Intake
import org.firstinspires.ftc.teamcode.motors.Wheels
import org.firstinspires.ftc.teamcode.pid.PidController
import org.firstinspires.ftc.teamcode.sensors.Encoder
import org.firstinspires.ftc.teamcode.utils.*
import kotlin.math.*

@Autonomous(name = "Blue-Path", group = "Path")
open class PathAuto : LinearOpMode() {

    private val robot = TeamRobot()
    private val wheels = Wheels()
    private val encoder = Encoder()
    private val intake = Intake()

    private val rotationPid = PidController(0.45, 0.0, 20.0)

    private lateinit var outtakeLeft: Servo
    private lateinit var outtakeRight: Servo
    private lateinit var gripper: Servo
    private lateinit var spinner: Servo
    private lateinit var foundationLeft: Servo
    private lateinit var foundationRight: Servo

    private var pointIndex = 0

    //private val path by fastLazy { PathParser.parseAsset(hardwareMap.appContext, "path/test.json") }
    private val path = listOf(
        PathPoint(
            cmX = -33.63255745421882,
            cmY = 57.86473642268101,
            angle = 0.5910196265432275,
            action = PathPoint.ACTION_START_INTAKE
        ),
        PathPoint(cmX = -15.19471073257481, cmY = 88.706576971581, angle = 0.6300877414469301),
        PathPoint(
            cmX = -36.90848409129329,
            cmY = 63.74020649035127,
            angle = 1.4416486364465586,
            action = PathPoint.ACTION_STOP_INTAKE
        ),
        PathPoint(cmX = -170.93058343702936, cmY = 49.668992425600656, angle = 1.5538550781699887),
        PathPoint(cmX = -186.9624951959268, cmY = 58.6962028677818, angle = 3.0154953265672746),
        PathPoint(cmX = -186.96085951630826, cmY = 58.6991936284283, angle = 3.0153545405676216),
        PathPoint(
            cmX = -166.87443051886447,
            cmY = 59.28812464224668,
            angle = 3.10785094233963,
            action = PathPoint.ACTION_ATTACH_FOUNDATION
        ),
        PathPoint(cmX = -164.37131504332135, cmY = 85.94570107167874, angle = 1.5299214582289846),
        PathPoint(
            cmX = -164.3707227235617,
            cmY = 85.948107473502,
            angle = 1.5298510652291581,
            action = PathPoint.ACTION_SIMPLE_OUTTAKE
        ),
        PathPoint(
            cmX = -189.578872776745,
            cmY = 71.37849193099981,
            angle = 1.5328075712218725,
            action = PathPoint.ACTION_DETACH_FOUNDATION
        ),
        PathPoint(cmX = -159.71579058646137, cmY = 56.863833536475035, angle = 1.561175950151949),
        PathPoint(cmX = -84.59913286877313, cmY = 59.56045321588649, angle = 1.5502650351788423)
    )

    override fun runOpMode() {
        robot.init(hardwareMap, listOf(wheels, intake, encoder), listOf(encoder))
        RobotPos.resetAll()

        rotationPid.setOutputRange(-0.3, 0.3)

        outtakeLeft = hardwareMap.getDevice("out_left")
        outtakeRight = hardwareMap.getDevice("out_right")
        gripper = hardwareMap.getDevice("gripper")
        spinner = hardwareMap.getDevice("spinner")
        foundationLeft = hardwareMap.getDevice("foundation_left")
        foundationRight = hardwareMap.getDevice("foundation_right")

        while (!isStarted) {
            telemetry.addData("Status", "Waiting for start")
            telemetry.addData("Path Size", path.size)
            telemetry.update()
        }
        robot.start()

        path.forEach {
            Log.v(TAG, "Reached Position $pointIndex")
            RobotPos.targetX = it.cmX
            RobotPos.targetY = it.cmY
            RobotPos.targetAngle = it.angle

            goToPoint()
            performAction(it.action)

            pointIndex++
        }

        intake.left.power = 0.0
        intake.right.power = 0.0

        robot.stop()
    }

    private fun goToPoint() {
        rotationPid.reset()

        val startTime = System.currentTimeMillis()

        while (opModeIsActive()) {
            val distanceToTargetX = RobotPos.targetX - RobotPos.currentX
            val distanceToTargetY = RobotPos.targetY - RobotPos.currentY
            val distance = hypot(distanceToTargetX, distanceToTargetY)
            val deltaAngle = RobotPos.targetAngle - RobotPos.currentAngle

            if (distance smaller 2.0 && deltaAngle smaller Math.toRadians(12.0))
                break

            val robotMovementAngle = atan2(distanceToTargetX, distanceToTargetY)
            val motionX = sin(robotMovementAngle) * MOTOR_SPEED
            val motionY = cos(robotMovementAngle) * MOTOR_SPEED

            rotationPid.setPoint = RobotPos.targetAngle
            val rotation = rotationPid.compute(RobotPos.currentAngle)
            movement(motionX, motionY, rotation)

            with(telemetry) {
                addData("Current X", RobotPos.currentX)
                addData("Current Y", RobotPos.currentY)
                addData("Current Angle", RobotPos.currentAngle)
                addData("--", "--")
                addData("Target X", RobotPos.targetX)
                addData("Target Y", RobotPos.targetY)
                addData("Target Angle", RobotPos.targetAngle)
                addData("--", "--")
                addData("MotionX", motionX)
                addData("MotionY", motionY)
                addData("PID Rotation", rotation)
                update()
            }

            if (System.currentTimeMillis() - startTime > 3000)
                break

            Thread.sleep(5)
        }
    }

    private fun movement(x: Double, y: Double, rotation: Double): Boolean {
        val magnitude = hypot(x, y) * MOTOR_SPEED

        if (magnitude smaller 0.05) return false

        val angle = atan2(y, x) - Math.PI / 2

        val speedX = magnitude * sin(angle + Math.PI / 4)
        val speedY = magnitude * sin(angle - Math.PI / 4)

        with(wheels) {
            rightFront.power = -speedX + rotation
            leftFront.power = -speedY + rotation
            rightBack.power = speedY + rotation
            leftBack.power = speedX + rotation
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
            PathPoint.ACTION_SIMPLE_OUTTAKE -> outtake()
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

    private fun outtake() {
        gripper.position = 0.75
        Thread.sleep(500)

        outtakeLeft.position = 0.0
        outtakeRight.position = 1.0
        Thread.sleep(500)

        spinner.position = 1.0
        Thread.sleep(500)
        gripper.position = 0.35
        Thread.sleep(500)

        /////
        gripper.position = 0.75
        spinner.position = 0.1
        Thread.sleep(500)

        outtakeLeft.position = 1.0
        outtakeRight.position = 0.0
        Thread.sleep(500)
    }

    private companion object {
        const val TAG = "PathAuto"
        const val MOTOR_SPEED = 0.6

        infix fun Double.smaller(other: Double) = abs(this) < other
    }
}
