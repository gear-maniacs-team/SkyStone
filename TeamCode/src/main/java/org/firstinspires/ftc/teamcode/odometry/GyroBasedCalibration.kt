package org.firstinspires.ftc.teamcode.odometry

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.RobotPos
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.motors.Wheels
import org.firstinspires.ftc.teamcode.sensors.Encoder
import org.firstinspires.ftc.teamcode.sensors.Gyro

@TeleOp(name = "Actually calibrate")
class GyroBasedCalibration : LinearOpMode() {

    private val robot = TeamRobot()
    private val wheels = Wheels()
    private val gyro = Gyro()
    private val encoder = Encoder()

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

        val difference = (encoder.right.currentPosition - -encoder.left.currentPosition) / RobotPos.currentAngle
        val result = (Encoder.ticksToCM(encoder.right.currentPosition.toDouble()) - Encoder.ticksToCM(-encoder.left.currentPosition.toDouble())) / RobotPos.currentAngle
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