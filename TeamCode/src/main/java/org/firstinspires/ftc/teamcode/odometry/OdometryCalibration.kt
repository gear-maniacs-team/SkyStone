package org.firstinspires.ftc.teamcode.odometry

import com.qualcomm.robotcore.eventloop.opmode.Disabled
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

@Disabled
@TeleOp(name = "Odometry System Calibration", group = "Odometry")
class OdometryCalibration : LinearOpMode() {

    private val robot = TeamRobot()
    private val wheels = Wheels()
    private val encoders = Encoder()
    private val gyro = Gyro()

    private var timer = ElapsedTime()

    private var horizontalTickOffset = 0.0

    private val wheelBaseSeparationFile: File = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt")
    private val horizontalTickOffsetFile: File = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt")

    override fun runOpMode() {
        robot.init(hardwareMap, listOf(wheels, encoders, gyro), listOf(gyro))

        wheels.setZeroPowerBehaviorAll(DcMotor.ZeroPowerBehavior.BRAKE)

        waitForStart()
        robot.start()

        //Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
        while (RobotPos.currentAngle < Math.PI / 2 && opModeIsActive()) {
            wheels.setVelocityAll(-PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED)

            if (RobotPos.currentAngle < 60) {
                wheels.setVelocityAll(-PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED)
            } else {
                wheels.setVelocityAll(-PIVOT_SPEED / 2, -PIVOT_SPEED / 2, PIVOT_SPEED / 2, PIVOT_SPEED / 2)
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

        val wheelBaseSeparation = 180 * verticalEncoderTickOffsetPerDegree / (Math.PI * COUNTS_PER_INCH)

        horizontalTickOffset = encoders.back.currentPosition / Math.toRadians(RobotPos.currentAngle)

        // Write the constants to text files
        wheelBaseSeparationFile.writeText(wheelBaseSeparation.toString())
        horizontalTickOffsetFile.writeText(horizontalTickOffset.toString())

        while (opModeIsActive()) {
            with(telemetry) {
                addData("Odometry System Calibration Status", "Calibration Complete")
                addData("Wheel Base Separation", wheelBaseSeparation)
                addData("Horizontal Encoder Offset", horizontalTickOffset)
                addData("IMU Angle", RobotPos.currentAngle)
                addData("Vertical Left Position", -encoders.left.currentPosition)
                addData("Vertical Right Position", encoders.right.currentPosition)
                addData("Horizontal Position", encoders.back.currentPosition)
                addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree)
                update()
            }
        }

        robot.stop()
    }

    private fun Wheels.setVelocityAll(rightFront: Double, rightBack: Double, leftFront: Double, leftBack: Double) {
        this.rightFront.velocity = getFrontVelocity(rightFront)
        this.rightBack.velocity = getBackVelocity(rightBack)
        this.leftFront.velocity = getFrontVelocity(leftFront)
        this.leftBack.velocity = getBackVelocity(leftBack)
    }

    private fun getFrontVelocity(power: Double) =
        Wheels.rpmToTps(MAX_RPM * power, FRONT_ENCODER_COUNT)

    private fun getBackVelocity(power: Double) =
        Wheels.rpmToTps(MAX_RPM * power, BACK_ENCODER_COUNT)

    companion object {
        private const val MAX_RPM = 223.0
        private const val FRONT_ENCODER_COUNT = 383.6
        private const val BACK_ENCODER_COUNT = 753.2

        const val PIVOT_SPEED = 0.5
        //The amount of encoder ticks for each inch the robot moves. THIS WILL CHANGE FOR EACH ROBOT AND NEEDS TO BE UPDATED HERE
        const val COUNTS_PER_INCH = 307.699557
    }
}