package org.firstinspires.ftc.teamcode.terminator

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamRobot
import net.gearmaniacs.teamcode.drive.Drive
import net.gearmaniacs.teamcode.hardware.sensors.Encoders
import net.gearmaniacs.teamcode.pid.PIDFController
import net.gearmaniacs.teamcode.utils.RobotClock
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

@Autonomous(name = "MotionProfilingBoogaloo")
class MotionProfilingBoogaloo : LinearOpMode() {

    private val robot = TeamRobot()
    private val wheels = Wheels()
    private val encoders = Encoders()
    private val yPID = PIDFController(PIDCoefficients(6.0, 0.0, 0.0), Drive.kV, clock = RobotClock)
    private val xPID = PIDFController(PIDCoefficients(6.0, 0.0, 0.0), Drive.kV, clock = RobotClock)
    private val aPID = PIDFController(PIDCoefficients(6.0, 0.0, 0.0), Drive.kV, clock = RobotClock)

    override fun runOpMode() {
        RobotPos.resetAll()
        robot.init(hardwareMap, listOf(wheels, encoders), listOf(encoders))

        wheels.setZeroPowerBehaviorAll(DcMotor.ZeroPowerBehavior.BRAKE)
        wheels.setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        wheels.setModeAll(DcMotor.RunMode.RUN_USING_ENCODER)

        waitForStart()
        RobotPos.resetAll()
        robot.start()

        RobotPos.targetY = 30.0
        RobotPos.targetX = 0.0
        RobotPos.targetAngle = 0.0

        val maxVel = 10.0

        val yMotionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
            MotionState(0.0, 0.0),
            MotionState(-RobotPos.targetY, 0.0),
            maxVel,
            80.0,
            40.0
        )
        val xMotionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
            MotionState(0.0,0.0),
            MotionState(RobotPos.targetX, 0.0),
            maxVel,
            80.0,
            40.0
        )
        val aMotionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
            MotionState(0.0,0.0),
            MotionState(RobotPos.targetAngle, 0.0),
            maxVel,
            80.0,
            40.0
        )

        val timeStart = RobotClock.seconds()
        yPID.reset()
        xPID.reset()
        aPID.reset()

        xPID.setOutputBounds(-Drive.MAX_VEL, Drive.MAX_VEL)
        yPID.setOutputBounds(-Drive.MAX_VEL, Drive.MAX_VEL)
        aPID.setOutputBounds(-Drive.MAX_VEL, Drive.MAX_VEL)

        while (opModeIsActive()) {
            val elapsedTime = RobotClock.seconds() - timeStart

            val yState = yMotionProfile[elapsedTime]
            val xState = xMotionProfile[elapsedTime]
            val aState = aMotionProfile[elapsedTime]

            yPID.targetPosition = yState.x
            xPID.targetPosition = xState.x
            aPID.targetPosition = aState.a

            val yResult = yPID.update(RobotPos.currentY, yState.v)
            val xResult = xPID.update(RobotPos.currentX, xState.v)
            val aResult = aPID.update(RobotPos.currentAngle, aState.v)

            movement( x = xResult, y = yResult, theta = 0.0)
            printTelemetry420(FtcDashboard.getInstance().telemetry, yState, xState, aState)

            val yError = abs(RobotPos.targetY - RobotPos.currentY)
            val xError = abs(RobotPos.targetX - RobotPos.currentX)
            val aError = abs(RobotPos.targetAngle - RobotPos.currentAngle)

            if (yError < 5.0 && xError < 5.0 && aError < 5.0)
                break
        }

        wheels.setPower(0.0)
        sleep(2000)
        robot.stop()
    }

    private fun movement(x: Double, y: Double, theta: Double) {
        val currentAngle = RobotPos.currentAngle
        val sinOrientation = sin(currentAngle)
        val cosOrientation = cos(currentAngle)

        val fieldOrientedX = x * cosOrientation - y * sinOrientation
        val fieldOrientedY = x * sinOrientation + y * cosOrientation

        with(wheels) {
            frontLeft.velocity = Drive.cmToTicks(fieldOrientedX - fieldOrientedY - theta)
            backLeft.velocity = Drive.cmToTicks(-fieldOrientedX - fieldOrientedY - theta)
            frontRight.velocity = Drive.cmToTicks(-fieldOrientedX - fieldOrientedY + theta)
            backRight.velocity = Drive.cmToTicks(fieldOrientedX - fieldOrientedY + theta)
        }
    }

    private fun printTelemetry420(
        telemetry : Telemetry,
        yState: MotionState,
        xState: MotionState,
        aState: MotionState
    ){
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
            addData("Velocity Angle", "%.3f", aState.v)
            addLine()
            addData("State X", xState)
            addData("State Y", yState)
            addData("State Rotation", aState)
            update()
        }
    }


}