package org.firstinspires.ftc.teamcode.odometry

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.teamcode.RobotPos
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.motors.Wheels
import org.firstinspires.ftc.teamcode.sensors.Encoder
import org.firstinspires.ftc.teamcode.sensors.Gyro
import java.io.File
import kotlin.math.abs

@TeleOp(name = "Odometry System Calibration", group = "Odometry")
class OdometryCalibration : LinearOpMode() {

    private val robot = TeamRobot()
    private val wheels = Wheels()
    private val encoders = Encoder()
    private val gyro = Gyro()
    private val timer = ElapsedTime()

    private var horizontalTickOffset = 0.0

    override fun runOpMode() {
        robot.init(hardwareMap, listOf(wheels, encoders, gyro), listOf(gyro))

        wheels.setZeroPowerBehaviorAll(DcMotor.ZeroPowerBehavior.BRAKE)

        waitForStart()
        robot.start()

        //Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
        while (RobotPos.currentAngle < Math.PI / 2 && opModeIsActive()) {
            wheels.setPowerAll(-PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED)

            if (RobotPos.currentAngle < 60) {
                wheels.setPowerAll(-PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED)
            } else {
                wheels.setPowerAll(-PIVOT_SPEED / 2, -PIVOT_SPEED / 2, PIVOT_SPEED / 2, PIVOT_SPEED / 2)
            }
            telemetry.addData("IMU Angle", RobotPos.currentAngle)
            telemetry.update()
        }

        wheels.setPowerAll(0.0)
        timer.reset()
        while (timer.milliseconds() < 1000 && opModeIsActive()) {
            telemetry.addData("IMU Angle", RobotPos.currentAngle)
            telemetry.update()
        }

        //Record IMU and encoder values to calculate the constants for the global position algorithm
        val angle = RobotPos.currentAngle
        val encoderDifference = abs(encoders.left.currentPosition) + abs(encoders.left.currentPosition).toDouble()

        val verticalEncoderTickOffsetPerDegree = encoderDifference / angle

        val wheelBaseSeparation = 180 * verticalEncoderTickOffsetPerDegree / (Math.PI * COUNTS_PER_CM)

        horizontalTickOffset = encoders.back.currentPosition / RobotPos.currentAngle

        while (opModeIsActive()) {
            with(telemetry) {
                addData("Odometry System Calibration Status", "Calibration Complete")
                addData("Wheel Base Separation", Encoder.ticksToCM(wheelBaseSeparation))
                addData("Horizontal Encoder Offset", Encoder.ticksToCM(horizontalTickOffset))
                addData("IMU Angle", RobotPos.currentAngle)
                addData("Left Position", -encoders.left.currentPosition)
                addData("Right Position", encoders.right.currentPosition)
                addData("Back Position", encoders.back.currentPosition)
                addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree)
                update()
            }
        }

        robot.stop()
    }

    private fun Wheels.setPowerAll(rightFront: Double, rightBack: Double, leftFront: Double, leftBack: Double) {
        this.rightFront.power = rightFront
        this.rightBack.power = rightBack
        this.leftFront.power = -leftFront
        this.leftBack.power = -leftBack
    }

    companion object {
        const val PIVOT_SPEED = 0.5
        const val COUNTS_PER_CM = 4096 / (7.2 * Math.PI)
    }
}