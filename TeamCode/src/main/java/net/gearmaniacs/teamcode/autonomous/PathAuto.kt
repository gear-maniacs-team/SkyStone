package net.gearmaniacs.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
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
import net.gearmaniacs.teamcode.utils.getDevice
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
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

    private val rotationPid = PidController(0.45, 0.0, 20.0)

    private lateinit var gripper: Servo
    private lateinit var spinner: Servo

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
        robot.init(hardwareMap, listOf(wheels, intake, foundation, outtake, encoder), listOf(encoder))
        RobotPos.resetAll()

        rotationPid.setOutputRange(-MOTOR_SPEED_ROTATION, MOTOR_SPEED_ROTATION)

        gripper = hardwareMap.getDevice("gripper")
        spinner = hardwareMap.getDevice("spinner")

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
        }

        robot.stop()
    }

    private fun goToPoint() {
        rotationPid.reset()

        val startTime = System.currentTimeMillis()

        while (opModeIsActive()) {
            val distanceToX = RobotPos.targetX - RobotPos.currentX
            val distanceToY = RobotPos.targetY - RobotPos.currentY
            val distanceToAngle = RobotPos.targetAngle - RobotPos.currentAngle
            val distance = hypot(distanceToX, distanceToY)

            if (distance smaller DISTANCE_ERROR && distanceToAngle smaller ANGLE_ERROR)
                break

            val robotMovementAngle = atan2(distanceToX, distanceToY)
            val motionX = sin(robotMovementAngle) * MOTOR_SPEED_MOVEMENT
            val motionY = cos(robotMovementAngle) * MOTOR_SPEED_MOVEMENT

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
                break // Skip this point if it takes more than 3 seconds to arrive

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
            rightBack.power = speedY + rotation
            leftBack.power = speedX + rotation
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
        val power = if (start) 1.0 else 0.0
        intake.left.power = power
        intake.right.power = -power
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
        private const val MOTOR_SPEED_MOVEMENT = 0.6
        private const val MOTOR_SPEED_ROTATION = 0.3

        private const val DISTANCE_ERROR = 1.5
        private val ANGLE_ERROR = Math.toRadians(5.0)

        private infix fun Double.smaller(other: Double) = abs(this) < other
    }
}
