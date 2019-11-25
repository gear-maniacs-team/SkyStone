package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.sensors.Gyro
import org.firstinspires.ftc.teamcode.utils.SimplePIDController
import org.firstinspires.ftc.teamcode.utils.fastLazy

@TeleOp(name = "TestingGyro")
class TestingGyro : OpMode() {

    private val robot = TeamRobot()
    private val gyro = Gyro()
    private val wheelMotors by fastLazy { robot.wheelsMotors }
    private val pid = SimplePIDController(1.0, 0.0, 0.0)‬

    private val velocityFront = velocityForRPM(220, 383.6)
    private val velocityBack = velocityForRPM(220, 753.2)

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
        val modifier = when {
            gamepad1.dpad_up -> 0.05
            gamepad1.dpad_down -> -0.05
            else -> 0.0
        }

        when {
            gamepad1.x -> pid.kp += modifier
            gamepad1.a -> pid.ki += modifier
            gamepad1.b -> pid.kd += modifier
        }

        with(wheelMotors) {
            rightFront.velocity = velocityFront * 0.4
            leftFront.velocity = velocityFront * 0.4
            rightBack.velocity = velocityBack * 0.4
            leftBack.velocity = velocityBack * 0.4
        }

        pid.computePID(currentAngle, Math.PI)

        telemetry.addData("Current Angle", currentAngle)
        telemetry.addData("PID", pid.toString())
        telemetry.update()
    }

    private fun velocityForRPM(rpm: Int, magic: Double) = rpm * (magic / 60)
}
