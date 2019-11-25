package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.sensors.Gyro
import org.firstinspires.ftc.teamcode.utils.fastLazy

@TeleOp(name = "TestingGyro")
class TestingGyro : OpMode() {

    private val robot = TeamRobot()
    private val gyro = Gyro()
    private val wheelMotors by fastLazy { robot.wheelsMotors }

    private val velocityFront = getVelocityForRpmAndEncoderCycles(220, 383.6)
    private val velocityBack = getVelocityForRpmAndEncoderCycles(220, 753.2)

    override fun init() {
        robot.init(hardwareMap)
        gyro.init(hardwareMap)
        gyro.waitForCalibration()
    }

    override fun start() {
        gyro.start()
    }

    override fun loop() {
        val currentAngle = gyro.angle.toDouble()

        with(wheelMotors) {
            rightFront.velocity = velocityFront * 0.4
            leftFront.velocity = velocityFront * 0.4
            rightBack.velocity = velocityBack * 0.4
            leftBack.velocity = velocityBack * 0.4
        }

        telemetry.addData("Current Angle", currentAngle)
        telemetry.update()
    }

    override fun stop() {
        robot.stop()
    }

    private fun getVelocityForRpmAndEncoderCycles(rpm: Int, encoder: Double) = rpm * (encoder / 60)
}