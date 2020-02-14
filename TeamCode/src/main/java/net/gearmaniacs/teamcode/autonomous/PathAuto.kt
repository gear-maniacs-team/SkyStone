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
import net.gearmaniacs.teamcode.hardware.motors.Intake
import net.gearmaniacs.teamcode.hardware.motors.Wheels
import net.gearmaniacs.teamcode.hardware.sensors.Encoders
import net.gearmaniacs.teamcode.hardware.sensors.Gyro
import net.gearmaniacs.teamcode.hardware.sensors.GyroEncoders
import net.gearmaniacs.teamcode.hardware.servos.FoundationServos
import net.gearmaniacs.teamcode.hardware.servos.OuttakeServos
import net.gearmaniacs.teamcode.utils.PathPoint
import net.gearmaniacs.teamcode.utils.RobotClock
import net.gearmaniacs.teamcode.utils.extensions.getDevice
import net.gearmaniacs.teamcode.utils.extensions.smaller
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

@Autonomous(name = "Path")
open class PathAuto : LinearOpMode() {

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

    private val axialCoefficients = PIDCoefficients(0.05, 0.0001, 0.0)
    private val xPid = PIDFController(axialCoefficients, Drive.kV, clock = RobotClock)
    private val yPid = PIDFController(axialCoefficients, Drive.kV, clock = RobotClock)
    private val rPid = PIDFController(PIDCoefficients(0.5, 0.0, 0.0), Drive.kV, clock = RobotClock)

    private lateinit var gripper: Servo
    private lateinit var spinner: Servo

    private val path = listOf(
        PathPoint(30.0, 30.0, 0.0)
//        PathPoint(60.0, 10.0, 0.0),
//        PathPoint(0.0, 0.0, 0.0)
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
                Drive.MAX_VEL,
                Drive.MAX_ACC,
                100.0
            )

            yProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                MotionState(RobotPos.currentY, 0.0, 0.0, 0.0),
                MotionState(RobotPos.targetY, 0.0, 0.0, 0.0),
                Drive.MAX_VEL,
                Drive.MAX_ACC,
                100.0
            )

            rProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                MotionState(-RobotPos.currentAngle, 0.0, 0.0, 0.0),
                MotionState(RobotPos.targetAngle, 0.0, 0.0, 0.0),
                Drive.MAX_VEL,
                Drive.MAX_ACC,
                100.0
            )

            goToPoint()
            performAction(point.action)
            Thread.sleep(1000)
        }

        wheels.setPowerAll(0.0)
        robot.stop()
        Thread.sleep(3000)
    }

    private fun goToPoint() {
        xPid.reset()
        yPid.reset()
        rPid.reset()

//        xPid.targetPosition = RobotPos.targetX
//        yPid.targetPosition = RobotPos.targetY
//        rPid.targetPosition = RobotPos.targetAngle

        val startOfMotion = RobotClock.seconds()

        while (opModeIsActive()) {
            val elapsedTime = RobotClock.seconds() - startOfMotion

            val xCorrection = xProfile[elapsedTime]
            val yCorrection = yProfile[elapsedTime]
            val rCorrection = rProfile[elapsedTime]

            xPid.targetPosition = xCorrection.x
            yPid.targetPosition = yCorrection.x
            rPid.targetPosition = rCorrection.x
//            xPid.setOutputBounds(-abs(xCorrection.v),abs(xCorrection.v))
//            yPid.setOutputBounds(-abs(yCorrection.v),abs(yCorrection.v))
//            rPid.setOutputBounds(-abs(rotationCorrection.v),abs(rotationCorrection.v))

            val x = xPid.update(RobotPos.currentX, xCorrection.v, xCorrection.a)
            val y = yPid.update(RobotPos.currentY, yCorrection.v, yCorrection.a)
            val rot = rPid.update(-RobotPos.currentAngle, rCorrection.v, rCorrection.a)
            movement(x, y, rot)

            printTelemetry(telemetry, xCorrection, yCorrection, rCorrection)
            printTelemetry(dashboard.telemetry, xCorrection, yCorrection, rCorrection)

            if (RobotPos.targetX - RobotPos.currentX smaller 1.0
                && RobotPos.targetY - RobotPos.currentY smaller 1.0
                && RobotPos.targetAngle - RobotPos.currentAngle smaller 0.1
            ) {
                break
            }
        }
    }

    private fun printTelemetry(
        telemetry: Telemetry,
        xCorrection: MotionState,
        yCorrection: MotionState,
        rCorrection: MotionState
    ) {
        with(telemetry) {
            addData("Current X", "%.3f", RobotPos.currentX)
            addData("Current Y", "%.3f", RobotPos.currentY)
            addData("Current Angle", "%.3f", RobotPos.currentAngle)
            addLine("--")
            addData("Target X", "%.3f", RobotPos.targetX)
            addData("Target Y", "%.3f", RobotPos.targetY)
            addData("Target Angle", "%.3f", RobotPos.targetAngle)
            addLine("--")
            addData("Profile X", xCorrection)
            addData("Profile Y", yCorrection)
            addData("Profile Rotation", rCorrection)
            update()
        }
    }

    private fun movement(x: Double, y: Double, rot: Double) {
        val denom = abs(x) + abs(y) + abs(rot)
        var scaleX = x
        var scaleY = y
        var scaleR = rot

        if (denom > 1.0) {
            scaleX /= denom
            scaleY /= denom
            scaleR /= denom
        }

        val currentAngle = RobotPos.currentAngle
        val sinOrientation = sin(currentAngle)
        val cosOrientation = cos(currentAngle)

        val fieldOrientedX = scaleX * cosOrientation - scaleY * sinOrientation
        val fieldOrientedY = scaleX * sinOrientation + scaleY * cosOrientation

        with(wheels) {
            leftFront.power = (-fieldOrientedX - fieldOrientedY - scaleR)
            leftRear.power = (fieldOrientedX - fieldOrientedY - scaleR)
            rightRear.power = (-fieldOrientedX - fieldOrientedY + scaleR)
            rightFront.power = (fieldOrientedX - fieldOrientedY + scaleR)
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
