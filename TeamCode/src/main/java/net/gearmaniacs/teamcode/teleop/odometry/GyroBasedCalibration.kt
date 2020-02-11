package net.gearmaniacs.teamcode.teleop.odometry

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamRobot
import net.gearmaniacs.teamcode.hardware.motors.Wheels
import net.gearmaniacs.teamcode.hardware.sensors.Encoders
import net.gearmaniacs.teamcode.hardware.sensors.Gyro

@Disabled
@TeleOp(name = "Actually calibrate")
class GyroBasedCalibration : LinearOpMode() {

    private val robot = TeamRobot()
    private val wheels = Wheels()
    private val gyro = Gyro()
    private val encoder = Encoders()

    override fun runOpMode() {
        robot.init(hardwareMap, listOf(wheels, gyro, encoder), listOf(gyro))
        RobotPos.resetAll()

        waitForStart()
        robot.start()

        wheels.setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        wheels.setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        wheels.setZeroPowerBehaviorAll(DcMotor.ZeroPowerBehavior.FLOAT)

        wheels.setPowerAll(0.5)

        Thread.sleep(1000)
        wheels.setPowerAll(0.0)
        Thread.sleep(1000)

        val difference = (encoder.right.currentPosition - (-encoder.left.currentPosition)) / RobotPos.currentAngle
        val result =
            (Encoders.toCm(encoder.right.currentPosition.toDouble()) - Encoders.toCm(-encoder.left.currentPosition.toDouble())) / RobotPos.currentAngle
        while (opModeIsActive()) {
            telemetry.addData("Right",encoder.right.currentPosition)
            telemetry.addData("Left", encoder.left.currentPosition)
            telemetry.addData("Gyro", RobotPos.currentAngle)
            telemetry.addData("Difference", difference)
            telemetry.addData("Result", result)
            telemetry.update()
        }

        robot.stop()
    }

}