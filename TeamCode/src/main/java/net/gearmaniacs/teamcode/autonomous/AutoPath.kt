package net.gearmaniacs.teamcode.autonomous

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamRobot
import net.gearmaniacs.teamcode.drive.Drive
import net.gearmaniacs.teamcode.drive.Drive.MAX_VEL
import net.gearmaniacs.teamcode.hardware.motors.Intake
import net.gearmaniacs.teamcode.hardware.motors.Wheels
import net.gearmaniacs.teamcode.hardware.sensors.GyroEncoders
import net.gearmaniacs.teamcode.hardware.servos.FoundationServos
import net.gearmaniacs.teamcode.hardware.servos.OuttakeServos
import net.gearmaniacs.teamcode.utils.PathPoint
import net.gearmaniacs.teamcode.utils.PerformanceProfiler
import net.gearmaniacs.teamcode.utils.RobotClock
import net.gearmaniacs.teamcode.utils.extensions.getDevice
import net.gearmaniacs.teamcode.utils.extensions.smaller
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

@Autonomous(name = "AutoPath")
open class AutoPath : LinearOpMode() {

    private val robot = TeamRobot()
    private val wheels = Wheels()
    private val encoder = GyroEncoders()
    private val intake = Intake()
    private val foundation = FoundationServos()
    private val outtake = OuttakeServos()
    private val dashboard = FtcDashboard.getInstance()

    private lateinit var xProfile: MotionProfile
    private lateinit var yProfile: MotionProfile
    private lateinit var rProfile: MotionProfile

    private val axialCoefficients = PIDCoefficients(5.0, 0.001, 0.1)
    private val xPid = PIDFController(axialCoefficients, Drive.kV, clock = RobotClock)
    private val yPid = PIDFController(axialCoefficients, Drive.kV, clock = RobotClock)
    private val rPid = PIDFController(PIDCoefficients(100.0, 1.0, 0.0), Drive.kV, clock = RobotClock)

    private lateinit var gripper: Servo
    private lateinit var spinner: Servo

    private val path = listOf(
        PathPoint(0.0, -60.0, 0.0, action = PathPoint.ACTION_ATTACH_FOUNDATION),
        PathPoint(-10.0, -20.0, Math.PI / 2, action = PathPoint.ACTION_DETACH_FOUNDATION),
        PathPoint(-20.0, -20.0, Math.PI / 2),
        PathPoint(60.0, -20.0, Math.PI / 2)
    )

    override fun runOpMode() {
        RobotPos.resetAll()
        robot.init(
            hardwareMap, listOf(
                wheels,
                intake,
                foundation,
                outtake,
                 encoder
            ), listOf(encoder)
        )

        gripper = hardwareMap.getDevice("gripper")
        spinner = hardwareMap.getDevice("spinner")
        wheels.setZeroPowerBehaviorAll(DcMotor.ZeroPowerBehavior.BRAKE)
        wheels.setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        wheels.setModeAll(DcMotor.RunMode.RUN_USING_ENCODER)

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

            xProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                MotionState(RobotPos.currentX, 0.0, 0.0, 0.0),
                MotionState(RobotPos.targetX, 0.0, 0.0, 0.0),
                MAX_VEL,
                Drive.MAX_ACC,
                Drive.MAX_JERK
            )

            yProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                MotionState(RobotPos.currentY, 0.0, 0.0, 0.0),
                MotionState(RobotPos.targetY, 0.0, 0.0, 0.0),
                MAX_VEL,
                Drive.MAX_ACC,
                Drive.MAX_JERK
            )

            rProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                MotionState(-RobotPos.currentAngle, 0.0, 0.0, 0.0),
                MotionState(RobotPos.targetAngle, 0.0, 0.0, 0.0),
                MAX_VEL,
                Drive.MAX_ACC,
                Drive.MAX_JERK
            )

            goToPoint()
            performAction(point.action)
            Thread.sleep(500)
        }

        wheels.setPowerAll(0.0)
        robot.stop()
        while (!isStopRequested);
    }

    private fun goToPoint() {
        xPid.reset()
        yPid.reset()
        rPid.reset()

        val performanceProfiler = PerformanceProfiler()
        val startOfMotion = RobotClock.seconds()

        while (opModeIsActive()) {
            val ms = performanceProfiler.update()
            val elapsedTime = RobotClock.seconds() - startOfMotion

            val xState = xProfile[elapsedTime]
            val yState = yProfile[elapsedTime]
            val rState = rProfile[elapsedTime]

            xPid.targetPosition = xState.x
            yPid.targetPosition = yState.x
            rPid.targetPosition = rState.x

            xPid.setOutputBounds(-MAX_VEL, MAX_VEL)
            yPid.setOutputBounds(-MAX_VEL, MAX_VEL)
            rPid.setOutputBounds(-MAX_VEL, MAX_VEL)

            val x = xPid.update(RobotPos.currentX, xState.v, xState.a)
            val y = yPid.update(RobotPos.currentY, yState.v, yState.a)
            val theta = rPid.update(RobotPos.currentAngle, rState.v, rState.a)
            movement(x, y, theta)

            printTelemetry(telemetry, xState, yState, rState, ms)
            printTelemetry(dashboard.telemetry, xState, yState, rState, ms)

            val xError = abs(RobotPos.targetX - RobotPos.currentX)
            val yError = abs(RobotPos.targetY - RobotPos.currentY)
            val thetaError = abs(RobotPos.targetAngle - RobotPos.currentAngle)

            if (xError < 2 && yError < 2 && thetaError < Math.toRadians(5.0)) {
                break
            }
        }
    }

    private fun printTelemetry(
        telemetry: Telemetry,
        xState: MotionState,
        yState: MotionState,
        rState: MotionState,
        ms: Double
    ) {
        with(telemetry) {
            addData("Ms/Update", ms.toFloat())
            addData("Current X", "%.3f", RobotPos.currentX)
            addData("Current Y", "%.3f", RobotPos.currentY)
            addData("Current Angle", "%.3f", RobotPos.currentAngle)
            addLine()
            addData("Target X", "%.3f", RobotPos.targetX)
            addData("Target Y", "%.3f", RobotPos.targetY)
            addData("Target Angle", "%.3f", RobotPos.targetAngle)
            addLine()
            addData("Velocity X", "%.3f", xState.v)
            addData("Velocity Y", "%.3f", yState.v)
            addData("Velocity Angle", "%.3f", rState.v)
            addLine()
            addData("State X", xState)
            addData("State Y", yState)
            addData("State Rotation", rState)
            update()
        }
    }

    private fun movement(x: Double, y: Double, theta: Double) {
        val currentAngle = RobotPos.currentAngle
        val sinOrientation = sin(currentAngle)
        val cosOrientation = cos(currentAngle)

        val fieldOrientedX = x * cosOrientation - y * sinOrientation
        val fieldOrientedY = x * sinOrientation + y * cosOrientation

        with(wheels) {
            leftFront.velocity = Drive.cmToTicks(-fieldOrientedX - fieldOrientedY - theta)
            leftRear.velocity = Drive.cmToTicks(fieldOrientedX - fieldOrientedY - theta)
            rightRear.velocity = Drive.cmToTicks(-fieldOrientedX - fieldOrientedY + theta)
            rightFront.velocity = Drive.cmToTicks(fieldOrientedX - fieldOrientedY + theta)
        }
    }

    private fun performAction(action: Int) {
        var a = action
        var i = 0
        while (action != 0) {
            val thisAction = action and (1 shl i)
            a = a and (1 shl i)
            when (thisAction) {
                PathPoint.ACTION_NONE -> return
                PathPoint.ACTION_START_INTAKE -> actionIntake(true)
                PathPoint.ACTION_STOP_INTAKE -> actionIntake(false)
                PathPoint.ACTION_ATTACH_FOUNDATION -> foundation(true)
                PathPoint.ACTION_DETACH_FOUNDATION -> foundation(false)
                PathPoint.ACTION_SIMPLE_OUTTAKE -> outtake()
                else -> throw IllegalArgumentException("Unsupported Path Action")
            }
            i++
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
}
