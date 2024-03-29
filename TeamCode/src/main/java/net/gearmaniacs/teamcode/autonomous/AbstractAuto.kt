package net.gearmaniacs.teamcode.autonomous

import android.util.Log
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamRobot
import net.gearmaniacs.teamcode.detector.OpenCvManager
import net.gearmaniacs.teamcode.detector.SkystoneDetector
import net.gearmaniacs.teamcode.drive.Drive
import net.gearmaniacs.teamcode.drive.Drive.MAX_VEL
import net.gearmaniacs.teamcode.hardware.motors.Intake
import net.gearmaniacs.teamcode.hardware.motors.Lift
import net.gearmaniacs.teamcode.hardware.motors.Wheels
import net.gearmaniacs.teamcode.hardware.sensors.GyroEncoders
import net.gearmaniacs.teamcode.hardware.servos.FoundationServos
import net.gearmaniacs.teamcode.hardware.servos.OuttakeServos
import net.gearmaniacs.teamcode.pid.PIDFController
import net.gearmaniacs.teamcode.utils.PathAction
import net.gearmaniacs.teamcode.utils.PathPoint
import net.gearmaniacs.teamcode.utils.RobotClock
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.robotcore.internal.ui.UILocation
import java.util.concurrent.Executors
import java.util.concurrent.Future
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

abstract class AbstractAuto : LinearOpMode() {

    private val pipeline = SkystoneDetector(telemetry)
    private val manager = OpenCvManager(pipeline)

    private val robot = TeamRobot(false)
    private val wheels = Wheels()
    private val encoder = GyroEncoders()
    private val intake = Intake()
    private val lift = Lift()
    private val foundation = FoundationServos()
    private val outtake = OuttakeServos()

    private val axialCoefficients = PIDCoefficients(7.3, 1.82, 0.0)
    private val xPid = PIDFController(axialCoefficients, Drive.kV, clock = RobotClock)
    private val yPid = PIDFController(axialCoefficients, Drive.kV, clock = RobotClock)
    private val rPid = PIDFController(PIDCoefficients(150.0, 25.0, 6.25), Drive.kV, clock = RobotClock)

    private val executor = Executors.newSingleThreadExecutor()
    private val asyncActions = mutableListOf<Future<*>>()

    open val usesDetector: Boolean = true
    open val isBluePath: Boolean = true

    abstract val pathLeft: List<PathPoint>
    abstract val pathCenter: List<PathPoint>
    abstract val pathRight: List<PathPoint>

    override fun runOpMode() {
        RobotPos.resetAll()

        if (usesDetector) {
            // Try starting OpenCv before starting the Gyro and the rest of the hardware
            if (!manager.tryInitAndStart(hardwareMap)) {
                AppUtil.getInstance().showToast(UILocation.BOTH, "Failed Initializing OpenCV")
                return
            }
        }

        telemetry.addLine("Finished Starting Camera")
        telemetry.addLine("Starting Gyro")
        telemetry.update()

        encoder.showUpdateTime = false
        robot.init(
            hardwareMap,
            listOf(
                wheels,
                intake,
                foundation,
                outtake,
                lift,
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

        encoder.start()

        while (!isStarted) {
            val position = pipeline.skystonePosition ?: SkystoneDetector.SkystonePosition.CENTER_STONE

            with(telemetry) {
                if (usesDetector)
                    addData("Skystone Position", position)
                addData("Status", "Waiting for start")
                addData("Path Size", getPath(position).size)
                update()
            }

            Thread.yield()
        }

        // Stop Detector
        val position =
            pipeline.skystonePosition.takeIf { usesDetector } ?: SkystoneDetector.SkystonePosition.CENTER_STONE
        if (usesDetector)
            manager.stop()

        robot.start()
        val elapsedTime = ElapsedTime()

        resetServos()

        getPath(position).forEach { point ->
            if (isStopRequested) return
            outtake.deactivateSpinner()
            asyncActions.forEach { it.get() }
            asyncActions.clear()

            RobotPos.targetX = point.x
            RobotPos.targetY = point.y
            RobotPos.targetAngle = point.angle

            if (!isBluePath) {
                RobotPos.targetY -= 11.0
                RobotPos.targetX *= -1.0
                RobotPos.targetAngle *= -1.0
            }

            val xProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                MotionState(RobotPos.currentX, 0.0, 0.0, 0.0),
                MotionState(RobotPos.targetX, 0.0, 0.0, 0.0),
                MAX_VEL,
                Drive.MAX_ACC,
                Drive.MAX_JERK
            )

            val yProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                MotionState(RobotPos.currentY, 0.0, 0.0, 0.0),
                MotionState(RobotPos.targetY, 0.0, 0.0, 0.0),
                MAX_VEL,
                Drive.MAX_ACC,
                Drive.MAX_JERK
            )

            val rProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                MotionState(RobotPos.currentAngle, 0.0, 0.0, 0.0),
                MotionState(RobotPos.targetAngle, 0.0, 0.0, 0.0),
                Drive.MAX_VEL_ANG,
                Drive.MAX_ACC_ANG,
                Drive.MAX_JERK_ANG
            )

            goToPoint(point, xProfile, yProfile, rProfile)
            performAction(point.action)
        }

        telemetry.addData("Elapsed Time", elapsedTime.seconds())
        telemetry.update()

        wheels.setPowerAll(0.0)
        robot.stop()
        sleep(500)
        wheels.setZeroPowerBehaviorAll(DcMotor.ZeroPowerBehavior.FLOAT)
        while (!isStopRequested)
            Thread.yield()
    }

    private fun goToPoint(
        point: PathPoint,
        xProfile: MotionProfile,
        yProfile: MotionProfile,
        rProfile: MotionProfile
    ) {
        xPid.reset()
        yPid.reset()
        rPid.reset()

        val startOfMotion = RobotClock.seconds()

        while (opModeIsActive()) {
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
            movement(x, y, theta)

            printTelemetry(telemetry, xState, yState, rState)
            //printTelemetry(FtcDashboard.getInstance().telemetry, xState, yState, rState)

            val xError = abs(RobotPos.targetX - RobotPos.currentX)
            val yError = abs(RobotPos.targetY - RobotPos.currentY)
            val thetaError = abs(RobotPos.targetAngle - RobotPos.currentAngle)

            if (xError < point.moveError && yError < point.moveError && thetaError < point.turnError)
                break
        }
    }

    private fun printTelemetry(
        telemetry: Telemetry,
        xState: MotionState,
        yState: MotionState,
        rState: MotionState
    ) {
        with(telemetry) {
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

    private fun getPath(position: SkystoneDetector.SkystonePosition) = when (position) {
        SkystoneDetector.SkystonePosition.LEFT_STONE -> pathLeft
        SkystoneDetector.SkystonePosition.CENTER_STONE -> pathCenter
        SkystoneDetector.SkystonePosition.RIGHT_STONE -> pathRight
    }

    private fun resetServos() {
        foundation.detach()
        outtake.deactivateSpinner()
        outtake.releaseGripper()
        outtake.retract()
    }

    private fun performAction(action: Int) {
        var index = 0
        var a = action

        while (a != 0) {
            val thisAction = (a and (1 shl index++))
            if (thisAction == PathAction.NO_ACTION)
                continue

            when (thisAction) {
                PathAction.SLEEP_500 -> sleep(500L)
                PathAction.START_INTAKE -> intake.setPowerAll(0.75)
                PathAction.STOP_INTAKE -> intake.setPowerAll(0.0)
                PathAction.ATTACH_FOUNDATION -> {
                    foundation.attach()
                    sleep(800)
                }
                PathAction.PREPARE_ATTACH_FOUNDATION -> foundation.prepareAttach()
                PathAction.DETACH_FOUNDATION -> foundation.detach()
                PathAction.EXTEND_OUTTAKE -> outtake(true)
                PathAction.RETRACT_OUTTAKE -> outtake(false)
                PathAction.ATTACH_GRIPPER -> gripper(true)
                PathAction.RELEASE_GRIPPER -> gripper(false)
                else -> Log.e("AbstractAuto", "Invalid PathAction")
            }

            a = a xor thisAction
        }
    }

    private fun outtake(extend: Boolean) {
        if (extend) {
            asyncActions += executor.submit {
                lift.setPowerAll(0.6)
                lift.setTargetPositionAll(400)
                sleep(400)
                outtake.extend()
                sleep(600)
                outtake.activateSpinner()
            }
        } else {
            // Full retract
            outtake.releaseGripper()
            lift.setPowerAll(0.3)
            lift.setTargetPositionAll(0)
            Thread.sleep(350)
            lift.setPowerAll(0.6)
            outtake.retract()
            outtake.deactivateSpinner()
            sleep(350)
        }
    }

    private fun gripper(attach: Boolean) {
        if (attach) {
            outtake.activateGripper()
        } else {
            asyncActions += executor.submit {
                outtake.releaseGripper()
                sleep(800)
            }
        }
    }
}
