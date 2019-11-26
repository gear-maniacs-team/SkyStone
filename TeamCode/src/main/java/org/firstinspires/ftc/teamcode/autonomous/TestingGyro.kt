package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.pid.PidController
import org.firstinspires.ftc.teamcode.pid.SimplePIDController
import org.firstinspires.ftc.teamcode.sensors.Gyro
import org.firstinspires.ftc.teamcode.utils.fastLazy
import org.firstinspires.ftc.teamcode.utils.getVelocityForRpmAndEncoderCycles
import java.lang.Thread.sleep

@TeleOp(name = "TestingGyro")
class TestingGyro : OpMode() {

    private val robot = TeamRobot()
    private val gyro = Gyro()
    private val wheelMotors by fastLazy { robot.wheelsMotors }
    private val pid: PidController = SimplePIDController(1.0, 0.01, 0.005)

    private val maxFrontVelocity = getVelocityForRpmAndEncoderCycles(220, 383.6)
    private val maxBackVelocity = getVelocityForRpmAndEncoderCycles(220, 753.2)

    override fun init() {
        robot.init(hardwareMap)
        gyro.init(hardwareMap)
        gyro.waitForCalibration()
    }

    override fun start() {
        gyro.start()
    }

    override fun loop() {
        //if (gamepad1.right_bumper)
        pid.target = Math.PI / 2
        val currentAngle = gyro.angle.toDouble()

        val modifier = when {
            gamepad1.dpad_up -> 0.05
            gamepad1.dpad_down -> -0.05
            else -> 0.0
        }

        when {
            gamepad1.x -> {
                pid.Kp += modifier
                if (modifier != 0.0) sleep(200)
            }
            gamepad1.a -> {
                pid.Ki += modifier
                if (modifier != 0.0) sleep(200)
            }
            gamepad1.b -> {
                pid.Kd += modifier
                if (modifier != 0.0) sleep(200)
            }
        }

        val result = pid.computePID(currentAngle)

        with(wheelMotors) {
            rightFront.velocity = maxFrontVelocity * (result / 100)
            leftFront.velocity = maxFrontVelocity * (result / 100)
            rightBack.velocity = maxBackVelocity * (result / 100)
            leftBack.velocity = maxBackVelocity * (result / 100)
        }

        telemetry.addData("PID", pid.toString())
        telemetry.addData("PID Result", result)
        telemetry.addData("Current Angle", currentAngle)
        telemetry.addData("Target Angle", pid.target)
        telemetry.update()
    }

    override fun stop() {
        robot.stop()
    }
}
