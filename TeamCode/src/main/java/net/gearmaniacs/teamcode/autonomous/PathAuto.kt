package net.gearmaniacs.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamRobot
import net.gearmaniacs.teamcode.hardware.motors.Intake
import net.gearmaniacs.teamcode.hardware.motors.Wheels
import net.gearmaniacs.teamcode.hardware.sensors.Encoders
import net.gearmaniacs.teamcode.hardware.servos.FoundationServos
import net.gearmaniacs.teamcode.hardware.servos.OuttakeServos
import net.gearmaniacs.teamcode.pid.PidController
import net.gearmaniacs.teamcode.utils.PathPoint
import net.gearmaniacs.teamcode.utils.extensions.getDevice
import net.gearmaniacs.teamcode.utils.extensions.smaller
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.sin

@Autonomous(name = "Blue-Path", group = "Path")
open class PathAuto : LinearOpMode() {

    private val robot = TeamRobot()
    private val wheels = Wheels()
    private val encoder = Encoders()
    private val intake = Intake()
    private val foundation = FoundationServos()
    private val outtake = OuttakeServos()

    private val xPid = PidController(0.6, 0.0001, 100.0)
    private val yPid = PidController(0.6, 0.0001, 100.0)
    private val rotationPid = PidController(0.45, 0.0, 15.0)

    private lateinit var gripper: Servo
    private lateinit var spinner: Servo

    private val path = listOf(
        PathPoint(30.0, 30.0, 0.0),
        PathPoint(60.0, 10.0, 0.0),
        PathPoint(0.0, 0.0, 0.0)
    )

    override fun runOpMode() {
        robot.init(hardwareMap, listOf(wheels, intake, foundation, outtake, encoder), listOf(encoder))
        RobotPos.resetAll()

        xPid.setOutputRange(-1.0, 1.0)
        yPid.setOutputRange(-1.0, 1.0)
        rotationPid.setOutputRange(-MOTOR_SPEED_ROTATION, MOTOR_SPEED_ROTATION)

        xPid.compute(0.0)
        yPid.compute(0.0)
        rotationPid.compute(0.0)

        gripper = hardwareMap.getDevice("gripper")
        spinner = hardwareMap.getDevice("spinner")
        wheels.setZeroPowerBehaviorAll(DcMotor.ZeroPowerBehavior.BRAKE)

        while (!isStarted) {
            telemetry.addData("Status", "Waiting for start")
            telemetry.addData("Path Size", path.size)
            telemetry.update()
        }
        robot.start()

        path.forEachIndexed { index, point ->
            telemetry.addData("Destination Index", index)

            RobotPos.targetX = point.cmX
            RobotPos.targetY = point.cmY
            RobotPos.targetAngle = point.angle

            goToPoint()
            performAction(point.action)
            Thread.sleep(1000)
        }

        wheels.setPowerAll(0.0)
        Thread.sleep(3000)

        robot.stop()
    }

    private fun goToPoint() {
        xPid.reset()
        yPid.reset()
        rotationPid.reset()

        xPid.setPoint = RobotPos.targetX
        yPid.setPoint = RobotPos.targetY
        rotationPid.setPoint = RobotPos.targetAngle

//        val startTime = System.currentTimeMillis()

        while (opModeIsActive()) {

//            if (distance smaller DISTANCE_ERROR && distanceToAngle smaller ANGLE_ERROR)
//                break

            val x = xPid.compute(RobotPos.currentX)
            val y = yPid.compute(RobotPos.currentY)
//            val rotation = rotationPid.compute(RobotPos.currentAngle)
            val rotation = 0.0
            movement(x, y, rotation)

            with(telemetry) {
                addData("Current X", RobotPos.currentX)
                addData("Current Y", RobotPos.currentY)
                addData("Current Angle", RobotPos.currentAngle)
                addData("--", "--")
                addData("Target X", RobotPos.targetX)
                addData("Target Y", RobotPos.targetY)
                addData("Target Angle", RobotPos.targetAngle)
                addData("--", "--")
                addData("PID X", x)
                addData("PID Y", y)
                addData("PID Rotation", rotation)
                update()
            }

            if (x smaller 0.15 && y smaller 0.15 && rotation smaller 0.15)
                break

//            if (System.currentTimeMillis() - startTime > 3000)
//                break // Skip this point if it takes more than 3 seconds to arrive

            Thread.sleep(5)
        }
    }

    private fun movement(x: Double, y: Double, rotation: Double) {
        val magnitude = hypot(x, y) * MOTOR_SPEED_MOVEMENT

        val angle = atan2(y, x) - Math.PI / 2

        val speedX = magnitude * sin(angle + Math.PI / 4)
        val speedY = magnitude * sin(angle - Math.PI / 4)

        with(wheels) {
            rightFront.power = -speedX + rotation
            leftFront.power = -speedY + rotation
            rightRear.power = speedY + rotation
            leftRear.power = speedX + rotation
        }
    }

    private fun performAction(action: Int) {
        when (action) {
            PathPoint.ACTION_NONE -> return
            PathPoint.ACTION_START_INTAKE -> actionIntake(true)
            PathPoint.ACTION_STOP_INTAKE -> actionIntake(false)
            PathPoint.ACTION_ATTACH_FOUNDATION -> foundation(true)
            PathPoint.ACTION_DETACH_FOUNDATION -> foundation(false)
            PathPoint.ACTION_SIMPLE_OUTTAKE -> outtake()
            else -> throw IllegalArgumentException("Unsupported Path Action")
        }
    }

    private fun actionIntake(start: Boolean) {
        val power = if (start) 0.8 else 0.0
        intake.setPowerAll(power)
    }

    private fun foundation(attach: Boolean) {
        if (attach) foundation.attach() else foundation.detach()
    }

    private fun outtake() {
        gripper.position = 0.75
        Thread.sleep(500)

        outtake.extend()
        Thread.sleep(500)

        spinner.position = 1.0
        Thread.sleep(500)
        gripper.position = 0.35
        Thread.sleep(500)

        /////
        gripper.position = 0.75
        spinner.position = 0.1
        Thread.sleep(500)

        outtake.retract()
        Thread.sleep(500)
    }

    private companion object {
        private const val TAG = "PathAuto"
        private const val MOTOR_SPEED_MOVEMENT = 0.5
        private const val MOTOR_SPEED_ROTATION = 0.4

        private const val DISTANCE_ERROR = 1.5
        private val ANGLE_ERROR = Math.toRadians(5.0)
    }
}
