package net.gearmaniacs.teamcode.autonomous

import android.util.Log
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamRobot
import net.gearmaniacs.teamcode.drive.Drive
import net.gearmaniacs.teamcode.drive.Drive.MAX_VEL
import net.gearmaniacs.teamcode.hardware.motors.Intake
import net.gearmaniacs.teamcode.hardware.motors.Wheels
import net.gearmaniacs.teamcode.hardware.sensors.GyroEncoders
import net.gearmaniacs.teamcode.hardware.servos.FoundationServos
import net.gearmaniacs.teamcode.hardware.servos.OuttakeServos
import net.gearmaniacs.teamcode.utils.PathAction
import net.gearmaniacs.teamcode.utils.PathPoint
import net.gearmaniacs.teamcode.utils.PerformanceProfiler
import net.gearmaniacs.teamcode.utils.RobotClock
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

abstract class AbstractAuto : LinearOpMode() {

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
    private val rPid = PIDFController(PIDCoefficients(150.0, 0.0, 6.0), Drive.kV, clock = RobotClock)

    abstract val path: List<PathPoint>

    override fun runOpMode() {
        RobotPos.resetAll()
        robot.init(
            hardwareMap,
            listOf(
                wheels,
                intake,
                foundation,
                outtake,
                encoder
            ),
            listOf(encoder)
        )

        wheels.setZeroPowerBehaviorAll(DcMotor.ZeroPowerBehavior.BRAKE)
        wheels.setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        wheels.setModeAll(DcMotor.RunMode.RUN_USING_ENCODER)

        xPid.setOutputBounds(-MAX_VEL, MAX_VEL)
        yPid.setOutputBounds(-MAX_VEL, MAX_VEL)
        rPid.setOutputBounds(-MAX_VEL, MAX_VEL)

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
                MotionState(RobotPos.currentAngle, 0.0, 0.0, 0.0),
                MotionState(RobotPos.targetAngle, 0.0, 0.0, 0.0),
                Drive.MAX_VEL_ANG,
                Drive.MAX_ACC_ANG,
                Drive.MAX_JERK_ANG
            )

            goToPoint(point)
            performAction(point.action)
            sleep(500)
        }

        wheels.setPowerAll(0.0)
        robot.stop()
        while (!isStopRequested);
    }

    private fun goToPoint(point: PathPoint) {
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

            val x = xPid.update(RobotPos.currentX, xState.v)
            val y = yPid.update(RobotPos.currentY, yState.v)
            val theta = rPid.update(RobotPos.currentAngle, rState.v)
            movement(x * point.moveSpeed, y * point.moveSpeed, theta * point.turnSpeed)

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
        var index = 0
        var a = action

        while (a != 0) {
            val thisAction = a and (1 shl index)
            if (thisAction == PathAction.NO_ACTION)
                continue

            when (thisAction) {
                PathAction.START_INTAKE -> actionIntake(true)
                PathAction.STOP_INTAKE -> actionIntake(false)
                PathAction.ATTACH_FOUNDATION -> foundation(true)
                PathAction.DETACH_FOUNDATION -> foundation(false)
                PathAction.EXTEND_OUTTAKE -> outtake(true)
                PathAction.RETRACT_OUTTAKE -> outtake(false)
                PathAction.ATTACH_GRIPPER -> gripper(true)
                PathAction.RELEASE_GRIPPER -> gripper(false)
                else -> Log.e("AbstractAuto", "Invalid PathAction")
            }

            a = a xor thisAction
            ++index
        }
    }

    private fun actionIntake(start: Boolean) {
        val power = if (start) 0.8 else 0.0
        intake.setPowerAll(power)
    }

    private fun foundation(attach: Boolean) {
        if (attach) foundation.attach() else foundation.detach()
    }

    private fun outtake(extend: Boolean) {
        if (extend) {
            outtake.extend()
            sleep(600)
            outtake.activateSpinner()
            sleep(1000)
            outtake.semiExtend()
        } else {
            // Full retract
            outtake.deactivateSpinner()
            sleep(400)
            outtake.retract()
        }
    }

    private fun gripper(attach: Boolean) {
        if (attach) outtake.activateGripper() else outtake.releaseGripper()
    }
}
